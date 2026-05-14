#uint8 MAXCHANNELS=72

int8 status
uint16 satellites_used                     # Number of satellites
int32[72]  satellite_used_prn       # PRN identifiers
uint16 satellites_visible
int32[72] satellite_visible_prn     # PRN identifiers
int32[72] satellite_visible_z       # Elevation of satellites
int32[72] satellite_visible_azimuth # Azimuth of satellites
int32[72]   satellite_visible_snr   # Signal-to-noise ratios (dB)
bool[72] satellite_visible_used

