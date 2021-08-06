import numpy as np
import pylas
from laspy import PointFormat
import struct
import math
from pyquaternion import Quaternion

imuFileName = "./7-28/imu_correct.imulog."
lasFileName = "./7-28/imu_correct.las"
gpsFileName = "./7-28/imu_correct.gpslog"

LLA0 = (0.0, 0.0, 0.0)
RPH0 = (0.0,0.0,0.0)
# Function Definitions

def adjustRotation(point, roll, pitch, heading):

    v1 = np.array([[point[0]], [point[1]], [point[2]]])

    # Convert to radians
    factor = math.pi / 180.0
    roll = roll * factor
    pitch = pitch * factor
    heading = (heading - 0) * factor
    #print(heading)
    # Construct rotation matrices
    rCos = math.cos(roll)
    rSin = math.sin(roll)
    pCos = math.cos(pitch)
    pSin = math.sin(pitch)
    hCos = math.cos(heading)
    hSin = math.sin(heading)
    rx = np.matrix([[1, 0, 0],
                   [0, rCos, rSin],
                   [0, -rSin, rCos]])

    ry = np.matrix([[pCos, 0, -pSin],
                   [0, 1, 0],
                   [pSin, 0, pCos]])

    rz = np.matrix([[hCos, hSin, 0],
                   [-hSin, hCos, 0],
                   [0, 0, 1]])

    i = np.array([[1,0,0]]).T
    j = np.array([[0,1,0]]).T
    k = np.array([[0,0,1]]).T
    i = rz * i
    j = rz * j
    k = rz * k
    r_test = np.matrix([[i[0,0],j[0,0],k[0,0]],
                        [i[1,0],j[1,0],k[1,0]],
                        [i[2,0],j[2,0],k[2,0]]])
    # Apply rotation
    r = np.transpose(rz) * (np.transpose(ry) * np.transpose(rx)) # z y x ****
    #return r
    v2 = r_test * v1
    return v2[0, 0], v2[1, 0], v2[2, 0]

def adjustTranslation(point, longitude, latitude, altitude): # X is E/W, Y is N/S. Z is altitude
    a = 6_378_137 # Semi-major axis for WGS-84
    b = 6_356_732.3142 #Semi-minor axis
    ab = a * b
    a2 = a * a
    b2 = b * b
    theta = (math.fabs(longitude)* (math.pi / 180) - LLA0[0])  # Change in longitude in rad. (Longitude final - longitude initial)
    phi0 = LLA0[1]
    phi1 = math.fabs(latitude) * (math.pi / 180)
    dY = theta * ab * math.sqrt(1/(b2 + a2*math.tan(phi0))) # dX = r*theta, r = radius of equatorial line at given latitude
    dX_r = (ab / math.sqrt(a2*math.pow(math.sin(phi0),2) + b2*math.pow(math.cos(phi0),2))) #Find radius of ellipse at given latitude
    dX = dX_r * math.sqrt(2-2*math.cos(phi1-phi0)) #Law of cosines
    print("Theta is " + str(theta))
    print(phi0)
    print(phi1)
    return dX, dY, altitude-LLA0[2]
def main():
    global LLA0
    global RPH0
    # Read IMU data into memory
    print("Reading IMU data into memory")
    struct_fmt = "Iffffff"
    struct_len = struct.calcsize(struct_fmt)
    struct_unpack = struct.Struct(struct_fmt).unpack_from
    imu_data = []
    imu_q = []
    with open(imuFileName, "rb") as f:
        while True:
            data = f.read(struct_len)
            if not data:
                break
            imu_data.append(struct_unpack(data))
            print(struct_unpack(data))

    # Read GPS data into memory
    print("Reading GPS data into memory")
    struct_fmt = "Ifff"
    struct_len = struct.calcsize(struct_fmt)
    struct_unpack = struct.Struct(struct_fmt).unpack_from
    gps_data = []
    with open(gpsFileName, "rb") as f:
        while True:
            data = f.read(struct_len)
            if not data:
                break
            gps_data.append(struct_unpack(data))

    # Read lidar data into memory
    print("Reading Lidar data into memory")
    las = pylas.read(lasFileName)
    points = las.points

    #Calculate rotation matrix for every point
    print("Calculating rotation matrices")
    for data in imu_data:
        pass
        #imu_q.append(Quaternion(matrix=adjustRotation(data[1], data[2], data[3])))


    # Use IMU data to adjust lidar data
    print("Using IMU data to adjust lidar data")
    index = 1
    length = len(imu_data)
    lengthPt = len(points)
    count = 0
    #print(length)
    RPH0 = (imu_data[0][1], imu_data[0][2], imu_data[0][3])
    for point in points:
        if (count % 10000 == 0):
            print(round(count/lengthPt * 100)) # Progress Counter
        time = point[9]  # PointFormat(1)
        while (index < length-1) and (imu_data[index][0] <= (time)): #Find imu_data and point at the same point in time
            index += 1

        t1 = float(time - imu_data[index-1][0])
        t2 = float(imu_data[index][0] - imu_data[index-1][0])
        #print("{0} | {1} | {2} = {3}/{4}. {5}".format(imu_data[index][0], time, t1/t2, t1, t2, imu_data[index-1][1] - imu_data[index][2]))

        #q1 = imu_q[index-1]#Quaternion(matrix=imu_q[index-1])
        #q2 = imu_q[index]#Quaternion(matrix=imu_q[index])
        #qSlerp = Quaternion.slerp(q1, q2, t1/t2)
        #qSlerp = qSlerp.normalised
        if not (index < length-1):
            point[0], point[1], point[2] = (0,0,0)
        else:
            #point[0], point[1], point[2], = qSlerp.rotate((point[0], point[1], point[2]))
            point[0], point[1], point[2] = adjustRotation(point, imu_data[index][1], imu_data[index][2], imu_data[index][3])
        count += 1;
        index -= 20 #Ensure correct imu data is selected--redo part of selection process every time

    # Apply GPS tranlsation
    print("Translating each point according to GPS location")
    index = 0
    length = len(gps_data)
    LLA0 = (gps_data[0][1] * (math.pi/180), gps_data[0][2] * (math.pi/180), gps_data[0][3]) #Define initial lat, long, and altitude
    for point in points:
        time = point[9]  # PointFormat(1)
        while (index < length) and (gps_data[index][0] <= time): #Find gps_data and point at the same point in time
            index += 1
        #point[0], point[1], point[2] = adjustTranslation(point, gps_data[index][1], gps_data[0][2], gps_data[0][3])

    # Apply RTK translation (later date)


    # Save File :-)
    las.points = points
    las.write("./7-28/correct_out_17.las")

def test1():
    las = pylas.read("./7-28/lidar_-22degrees.las")
    points = las.points
    #print(adjustTranslation())
    for point in points:
        #print(point)
        point[0], point[1], point[2] = adjustRotation(point, 0, 0, -45.0/2.0)
        #break

    las.write("./7-28/out_-22degrees.las")

def test2():
    global LLA0
    LLA0= (85.347834 * (math.pi/180), 34.994039 * (math.pi/180), 0)
    print(adjustTranslation(0, -85.345386, 35.005288, 0))

def combine():
    las1 = pylas.read("./7-28/out_45degrees.las")
    las2 = pylas.read("./7-28/lidar_0degrees.las")
    las3 = pylas.read("./7-28/out_22degrees.las")
    las4 = pylas.read("./7-28/out_66degrees.las")
    las5 = pylas.read("./7-28/out_-66degrees.las")
    las6 = pylas.read("./7-28/out_-22degrees.las")
    las7 = pylas.read("./7-28/out_-45degrees.las")
    points_final = np.array([], dtype=PointFormat(1).dtype())
    points2 = las2.points
    points1 = las1.points
    points3 = las3.points
    points4 = las4.points
    points7 = las7.points
    points6 = las6.points
    points5 = las5.points
    '''
    length = len(points1) + len(points2)
    pointcld = np.zeros(length, PointFormat(1).dtype())
    for point in points1:
        np.append(points_final, point)

    for point in points2:
        np.append(points_final, point)

    for x in points_final:
        print(x)
    las2.points = points_final
    print("sup")
    '''

    las2.points = np.concatenate((points1,points2,points3,points4, points5, points6, points7))
    las2.write("./7-28/las_combined_full_FULL.las")

if __name__ == "__main__":
    main()