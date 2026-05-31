import math

def gps_to_local(lat, lon, ref_lat, ref_lon):
    """
    Convert GPS (lat, lon) to local Cartesian coordinates (x, y) in meters.
    Using an approximation with the Haversine formula for distance.
    """
    R = 6378137.0  # Earth radius in meters
    
    # Convert to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)
    
    # Delta
    dlat = lat_rad - ref_lat_rad
    dlon = lon_rad - ref_lon_rad
    
    # Local coordinates approximation
    x = R * dlon * math.cos(ref_lat_rad)
    y = R * dlat
    
    return x, y

def local_to_gps(x, y, ref_lat, ref_lon):
    """
    Convert local Cartesian coordinates (x, y) to GPS (lat, lon).
    """
    R = 6378137.0
    ref_lat_rad = math.radians(ref_lat)
    
    dlat = y / R
    dlon = x / (R * math.cos(ref_lat_rad))
    
    lat = ref_lat + math.degrees(dlat)
    lon = ref_lon + math.degrees(dlon)
    
    return lat, lon
