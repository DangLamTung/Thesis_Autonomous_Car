import numpy as np
Lon_Or = 106.65644
Lat_Or = 10.77654

LatOrigin = 10.773390000000006
LonOrigin = 106.65971499999999

def FindMetersPerLat(lat): # Compute lengths of degrees
	
	m1 = 111132.92
	m2 = -559.82
	m3 = 1.175
	m4 = -0.0023
	p1 = 111412.84
	p2 = -93.5
	p3 = 0.118

	lat = np.deg2rad(lat)
	# Calculate the length of a degree of latitude and longitude in meters
	metersPerLat = m1 + (m2 * np.cos(2 * lat)) + (m3 * np.cos(4 * lat)) + (m4 * np.cos(6 * lat))
	metersPerLon = (p1 * np.cos(lat)) + (p2 * np.cos(3 * lat)) + (p3 * np.cos(5 * lat))
	return metersPerLat,metersPerLon
def ConvertGPStoUCS(_LatOrigin,_LonOrigin,Lat, Lon):
	metersPerLat,metersPerLon =  FindMetersPerLat(_LatOrigin)
	yPosition  = metersPerLat * (Lat - _LatOrigin)
	xPosition  = metersPerLon * (Lon - _LonOrigin)
	return xPosition, yPosition

def ConvertUCStoGPS(LatOrigin,LonOrigin,x, y):
	metersPerLat,metersPerLon =  FindMetersPerLat(LatOrigin)
	Lat_dis = ((LatOrigin + (y) / metersPerLat))
	Lon_dis = ((LonOrigin + (x) / metersPerLon))
	return Lat_dis,Lon_dis

def get_map_loc(pitch, x, y, grid_x, grid_y):
    pitch = pitch/180*np.pi
    if(grid_y > 10):
        loc_x = 10*(grid_x/6.0)*np.cos(pitch) + x - 10*(grid_y/10.0 - 1)*np.sin(-pitch)
        loc_y = 10*(grid_x/6.0)*np.sin(pitch) + y - 10*(grid_y/10.0 - 1)*np.cos(-pitch)
    else:
        loc_x = -10*(grid_x/6.0)*np.cos(pitch) + x - 10*(grid_y/10.0)*np.sin(-pitch)
        loc_y = -10*(grid_x/6.0)*np.sin(pitch) + y - 10*(grid_y/10.0)*np.cos(-pitch)
    
    return (int(loc_x),int(loc_y)), (loc_x,loc_y)

def get_map_pos(pitch, x, y):
    pitch = pitch/180*np.pi
    b_dl_x = 10*np.cos(pitch) + x
    b_dl_y = 10*np.sin(pitch) + y

    b_dr_x = -10*np.cos(pitch) + x
    b_dr_y = -10*np.sin(pitch) + y

    b_ul_x = 10*np.cos(pitch) + x - 10*np.sin(-pitch)
    b_ul_y = 10*np.sin(pitch) + y - 10*np.cos(-pitch)

    b_ur_x = -10*np.cos(pitch) + x - 10*np.sin(-pitch)
    b_ur_y = -10*np.sin(pitch) + y - 10*np.cos(-pitch)
    return (int(b_dl_x) ,int(b_dl_y)), (int(b_dr_x) ,int(b_dr_y)), (int(b_ul_x) ,int(b_ul_y)),(int(b_ur_x),int(b_ur_y))

def pix2global(x,y,size_x,size_y):
    lat_x = x*(106.66226 - 106.65644)/size_x + 106.65644
    lat_y = y*(10.77654 - 10.77023)/size_y + 10.77023
    return lat_x, lat_y
    
def global2pix(Lat,Lon,Lat_Or,Lon_Or):
    x, y = int((Lon - Lon_Or + 100)*map_png.shape[1]/(106.66226 - 106.65644)),int((Lat_Or - Lat)*map_png.shape[0]/(10.77654 - 10.77023))
    return x,y
