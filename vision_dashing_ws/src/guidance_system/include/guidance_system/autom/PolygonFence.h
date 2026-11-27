#ifndef GUIDANCE_SYSTEM_POLYGON_FENCE_H
#define GUIDANCE_SYSTEM_POLYGON_FENCE_H

// Horizontal geofence: a convex polygon shrunk inward by a margin.
//
// The polygon is the mocap lab boundary from
// mocap_boundaries_check/config/mocap_boundaries.yaml (world frame, ENU, [m]).
// Setpoints are kept inside the shrunk polygon, so the drone stays `margin`
// away from the walls that mocap_boundaries_check watches.

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

class PolygonFence {
public:
    using Point = std::array<double, 2>;

    bool empty() const { return _in.empty(); }
    const std::vector<Point>& vertices() const { return _in; }

    /// Set the boundary (vertices in order, either direction) and shrink it
    /// inward by `margin`. Throws if the polygon isn't convex or the margin
    /// leaves nothing.
    void set(const std::vector<Point>& outer, double margin) {
        const size_t n = outer.size();
        if (n < 3) throw std::runtime_error("geofence polygon needs at least 3 vertices");
        if (margin < 0.0) throw std::runtime_error("geofence margin must be >= 0");

        const double area2 = signedArea2(outer);
        if (std::fabs(area2) < 1e-9) throw std::runtime_error("geofence polygon has no area");
        const double s = area2 > 0.0 ? 1.0 : -1.0;   // +1 counter-clockwise

        for (size_t i = 0; i < n; ++i)
            if (s * cross(outer[i], outer[(i + 1) % n], outer[(i + 2) % n]) <= 0.0)
                throw std::runtime_error("geofence polygon must be convex (check vertex " +
                                         std::to_string((i + 1) % n) + ")");

        // Shift every edge inward by `margin`, then intersect neighbouring edges.
        std::vector<Point> base(n), dir(n);
        for (size_t i = 0; i < n; ++i) {
            const Point& a = outer[i];
            const Point& b = outer[(i + 1) % n];
            const double dx = b[0] - a[0], dy = b[1] - a[1], len = std::hypot(dx, dy);
            const double nx = -s * dy / len, ny = s * dx / len;   // inward normal
            base[i] = {a[0] + margin * nx, a[1] + margin * ny};
            dir[i]  = {dx, dy};
        }
        std::vector<Point> in(n);
        for (size_t i = 0; i < n; ++i) {
            const size_t p = (i + n - 1) % n;
            const double den = dir[p][0] * dir[i][1] - dir[p][1] * dir[i][0];
            const double t = ((base[i][0] - base[p][0]) * dir[i][1] - (base[i][1] - base[p][1]) * dir[i][0]) / den;
            in[i] = {base[p][0] + t * dir[p][0], base[p][1] + t * dir[p][1]};
        }
        // A margin too large for the polygon flips or collapses the result.
        if (s * signedArea2(in) <= 0.0)
            throw std::runtime_error("geofence margin is too large for the polygon");
        for (size_t i = 0; i < n; ++i)
            if (s * cross(in[i], in[(i + 1) % n], in[(i + 2) % n]) <= 0.0)
                throw std::runtime_error("geofence margin is too large for the polygon");

        _in = in;
        _sign = s;
    }

    /// Inside (or on the edge of) the shrunk polygon.
    bool contains(double x, double y) const {
        const size_t n = _in.size();
        for (size_t i = 0; i < n; ++i)
            if (_sign * cross(_in[i], _in[(i + 1) % n], Point{x, y}) < 0.0) return false;
        return true;
    }

    /// Move (x, y) to the nearest point of the shrunk polygon (unchanged if inside).
    /// Returns true if it moved.
    bool clamp(double& x, double& y) const {
        if (contains(x, y)) return false;
        double bx = x, by = y;
        nearestOnBoundary(x, y, bx, by);
        x = bx; y = by;
        return true;
    }

    /// Outward unit normal of the shrunk-polygon edge nearest to (x, y).
    void outwardNormal(double x, double y, double& nx, double& ny) const {
        const size_t i = nearestEdge(x, y);
        const Point& a = _in[i];
        const Point& b = _in[(i + 1) % _in.size()];
        const double dx = b[0] - a[0], dy = b[1] - a[1], len = std::hypot(dx, dy);
        nx = _sign * dy / len;
        ny = -_sign * dx / len;
    }

private:
    std::vector<Point> _in;
    double _sign = 1.0;

    static double cross(const Point& a, const Point& b, const Point& c) {
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    }
    static double signedArea2(const std::vector<Point>& v) {
        double a = 0.0;
        for (size_t i = 0; i < v.size(); ++i) {
            const Point& p = v[i];
            const Point& q = v[(i + 1) % v.size()];
            a += p[0] * q[1] - q[0] * p[1];
        }
        return a;
    }
    static void projectOnSegment(double x, double y, const Point& a, const Point& b, double& px, double& py) {
        const double dx = b[0] - a[0], dy = b[1] - a[1];
        const double l2 = dx * dx + dy * dy;
        double t = l2 > 0.0 ? ((x - a[0]) * dx + (y - a[1]) * dy) / l2 : 0.0;
        t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
        px = a[0] + t * dx;
        py = a[1] + t * dy;
    }
    size_t nearestEdge(double x, double y) const {
        size_t best = 0;
        double bestD = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < _in.size(); ++i) {
            double px, py;
            projectOnSegment(x, y, _in[i], _in[(i + 1) % _in.size()], px, py);
            const double d = std::hypot(x - px, y - py);
            if (d < bestD) { bestD = d; best = i; }
        }
        return best;
    }
    void nearestOnBoundary(double x, double y, double& bx, double& by) const {
        const size_t i = nearestEdge(x, y);
        projectOnSegment(x, y, _in[i], _in[(i + 1) % _in.size()], bx, by);
    }
};

#endif
