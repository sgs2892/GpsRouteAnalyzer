"""
Title : GPS_Project_Group3.py
Contributors: Swapnil Shah  [sss4174@rit.edu]
              Srujan Shetty [sgs2892@rit.edu]
"""

"""
The objective of this project is to read the gps data recorded by the tracker, then process the data and 
return the fastest path from Professor Kinsman's house to RIT.
In this project, we are trying to reduce the cost functions by minimizing three attributes:
    1-> Total time spent on stop signs, traffic lite and car parked other than at destination.
    2-> Total time spent when speed is over 60 mph.
    3-> Total number of turns i.e. left or right turns on the overall route.
Below are brief idea on our approach to achieve above mentioned goals.
    1-> We are monitoring the speed recorded by GPS tracker and a flag is raise whenever the speed falls below a 
        mentioned threshold. Then the time is recorded until the speed rises over a pickup threshold. Depending 
        on the time spent at a spot, we determine if the car is at a stop sign, traffic signal, or is parked.
    2-> In this case, instead of performing the laborious work of looking after every speed and keeping track 
        of every time speed went over 60 mph, we approximate all this by only recording the highest spped achieved
        in a trip and dividing the value by 60.
    3-> For the sole purpose of execution of this project, here we assume that before every left or right turn,
        the car will be slowed down to a certain speed. Here we are keeping the threshold similar to the one kept
        for stop signs above. Then at these points we find the bearing angles between two points and determine if
        the car has taken a turn or not.
Detailed explanation of the approach is mentioned on every function call and also the report submitted with the code. 
"""


import sys
import math
from haversine import haversine

def make_Modifications(degree_Value):
    """
    This method is called to convert the latitude and longitude co-ordinates in the form of
    degree minutes seconds.
    :param degree_Value: Co-ordinates from gps file
    :return:
    """
    degree_Value_degree = degree_Value // 100   # Get degree value i.e. first 2 digits
    degree_Value_temp = str(degree_Value).split(".")
    degree_Value_minute = float(degree_Value_temp[0][-2:] + "." + degree_Value_temp[1]) / 60  # get minute values
    return degree_Value_degree + degree_Value_minute


def get_coordinates(line):
    """
    This method converts the latitude and longitude from $GPRMC line of gps data and converts it into a format
    that can be stored in kml format file and read by Google Earth.
    Direction south and west can be differentiated by the negative sign in front of the co-ordinate value.
    :param line: $GPRMC line
    :return:
    """
    latitude = line[3]  # get latitude value
    latitude_direction = "" if line[4] == 'N' else "-" # get direction N or S
    longitude = line[5] # get longitude value
    longitude_direction = "" if line[4] == 'E' else "-"    # get direction E or W
    latitude = round(make_Modifications(float(latitude)), 6)
    longitude = round(make_Modifications(float(longitude)), 6)
    return (float(longitude_direction+str(longitude)), float(latitude_direction+str(latitude)), float(line[7]), float(line[1]))


def getSeconds(currentTime):
    """
    Converting the time into format of seconds that can be used to find total travel duration or keeping track
    of specific points on the route for further processing.
    :param currentTime: Recorded time in GPS data
    :return:
    """
    hh = int((str(currentTime))[:2]) # hh * 3600 seconds
    mm = int((str(currentTime))[2:4]) # mm * 60 seconds
    ss = float((str(currentTime))[4:])

    return hh * 3600 + mm * 60 + ss


def calculateAngle(startPoint, endPoint):
    """
    The method helps us to figure out whether the car took a turn or not between the start position and end position.
    We find the bearing angle between the two co-ordinates.
    We are making use of the standard formula that we have read for calculating the bearing angle.
    The concept for figuring out the direction where the car is exactly moving towards is used from the code we
    found as a reference. Reference is mentioned near the part which is been used.
    :param startPoint: Co-ordinates of either stop or traffic light
    :param endPoint: Readings from GPS data which are 75 counts away from startPoint
    :return: Direction where the car is moving
    """
    startLat = math.radians(startPoint[1])  # Converting co-ordinates from degree to radians
    endLat = math.radians(endPoint[1])
    diffLong = math.radians(endPoint[0] - startPoint[0])
    x = math.sin(diffLong) * math.cos(endLat)
    y = math.cos(startLat) * math.sin(endLat) - (math.sin(startLat)
                                           * math.cos(endLat) * math.cos(diffLong))
    initial_bearing = math.degrees(math.atan2(x, y))

    # Following concept is made used from reference we found at
    # "https://stackoverflow.com/questions/3209899/determine-compass-direction-from-one-lat-lon-to-the-other"
    bearings = ["NE", "E", "SE", "S", "SW", "W", "NW", "N"]
    index = initial_bearing - 22.5
    if index < 0:
        index += 360
    index = int(index / 45)
    return bearings[index]


def get_Stop_Signs(co_ordinates_list):
    """
    This is considered as the most important method of the entire project.
    Here we read the spped from gps file and then perform the following operation based on their values.
    Initially we have mentioned a initialPickUpSpeed which is to iterate through all the co-ordinates at start of trip
    There is a upperBound mentioned which raises a flag if the speed falls below this speed.
    Until the time, the car is in the range of upperBound and lowerBound, time is recorded.
    Noe once the car goes over the pickUp speed, we know that car is no longer in stop state.
    Based on the time calculated above, we predict where the car was at a stop sign or traffic light or was parked.
    For every stop sign and traffic signal, the condition is checked if the car took a turn from this point or not.
    :param co_ordinates_list:
    :return:
    """
    iterator = 0
    upperBound = 10.00
    lowerBound = 0.03
    pickUpSpeed = 1.0
    initialPickUpSpeed = 2.0 # To iterate through multiple lines at start of each trip
    stopSignTimeLimit = 6 # 6 seconds
    stopSign = []
    trafficLight = []
    parked = []
    turn = []
    maxSpeed = 0
    distanceLimit = 0.1

    while co_ordinates_list[iterator][2] < initialPickUpSpeed:
        # Whenever the trip is started, until the car reaches a initialPickUpSpeed, the co-ordinates are skipped.
        iterator += 1

    while iterator < len(co_ordinates_list):
        currentSpeed = co_ordinates_list[iterator][2]
        # Longitude and Latitude values
        stopLong = co_ordinates_list[iterator][0]
        stopLat = co_ordinates_list[iterator][1]
        stopStartTime = "000000"
        stopEndTime = "000000"
        checkTurn = False
        startDistance = (0, 0)
        if currentSpeed > maxSpeed:
            # Keeping track of maxSpeed for purpose of cost function
            maxSpeed = currentSpeed

        if currentSpeed < upperBound:
            # If currentSpeed falls below the upperBound, we suspect that the car might come to a stop.
            while currentSpeed < upperBound and currentSpeed > lowerBound:
                # Until car is between upperBound and lowerBound keep on iterating over.
                iterator += 1
                currentSpeed = co_ordinates_list[iterator][2]
            if currentSpeed < lowerBound:
                # Condition where the car comes to a halt.
                # Record the time when car came to halt.
                stopStartTime = co_ordinates_list[iterator][3]
                startDistance = (co_ordinates_list[iterator][1], co_ordinates_list[iterator][0])
                while currentSpeed < pickUpSpeed:
                    # Until car regains the speed to move, store the co-ordinate and time values.
                    stopEndTime = co_ordinates_list[iterator][3]
                    iterator += 1
                    if iterator >= len(co_ordinates_list):
                        break
                    currentSpeed = co_ordinates_list[iterator][2]
                    stopLong = co_ordinates_list[iterator][0]
                    stopLat = co_ordinates_list[iterator][1]
        # TIme in seconds for which the car was in halt state
        timeSpent = getSeconds(stopEndTime) - getSeconds(stopStartTime)
        if iterator >= len(co_ordinates_list):
            endDistance = (co_ordinates_list[iterator - (iterator + 1 - len(co_ordinates_list))][1], co_ordinates_list[iterator - (iterator + 1 - len(co_ordinates_list))][0])
        else:
            endDistance = (co_ordinates_list[iterator][1], co_ordinates_list[iterator][0])
        if haversine(startDistance, endDistance, unit='mi') < distanceLimit:
            # Findin the distance between two points.
            # Making sure that it is not a moving traffic when the speed is between upper and lower bound
            if timeSpent > 60:
                # If car is standing for more than 60 seconds, probably a parking
                parked.append((stopLong, stopLat, 0.0))
            elif timeSpent > stopSignTimeLimit:
                # If car is standing for more than 6 seconds, probably a traffic
                trafficLight.append((stopLong, stopLat, 0.0))
                checkTurn = True
            elif timeSpent > 1:
                stopSign.append((stopLong, stopLat, 0.0))
                checkTurn = True

        if checkTurn:
            if iterator + 10 < len(co_ordinates_list):
                # Checking if taking a turn from current location
                # Using current point and 10 points over
                currentAngle =  calculateAngle((co_ordinates_list[iterator][0], co_ordinates_list[iterator][1]), (stopLong, stopLat))
                nextAngle = calculateAngle((stopLong, stopLat), (co_ordinates_list[iterator+10][0], co_ordinates_list[iterator+10][1]))

                if currentAngle != nextAngle:
                    # If the directions where the car is heading towards is not equal, probably car took a turn
                    if (stopLong, stopLat, 0.0) not in parked:
                        turn.append((stopLong, stopLat, 0.0))
        iterator += 1
    return stopSign, trafficLight, parked, turn, maxSpeed


def outputKML(co_ordinates, stopSign, trafficLight, parked, turn, fileName):
    """
    This is the method that once the entire process is completed will create a kml file.
    Here two types of kml files are generated for every gps file.
    First-> kml file without reference to stop signs using pinpoints.
    Second-> Hazard kml file with reference to stop signs, turns, traffic lights.
    :param co_ordinates: List of co-ordinates read from GPS file
    :param stopSign: List of co-ordinates where we predict there is a stop sign
    :param trafficLight: List of co-ordinates where we predict there is a traffic light
    :param parked: List of co-ordinates where we predict car is parked
    :param turn: List of co-ordinates where we predict car took a turn (either left or right)
    :param fileName: File name to store the result as
    :return:
    """

    # Header of the kml file
    writeStr = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n " \
               "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n " \
               "<Document id=\"1\">\n " \
               "<Style id=\"4\">\n " \
               "<LineStyle id=\"5\">\n " \
               "<color>ff00ffff</color>\n " \
               "<colorMode>normal</colorMode>\n " \
               "<width>6</width> \n" \
               "</LineStyle> \n" \
               "</Style> \n" \
               "<Placemark id=\"3\"> \n" \
               "<name>GPS_Project_Group3</name> \n" \
               "<description>Speed in Knots, instead of altitude.</description> \n" \
               "<styleUrl>#4</styleUrl> \n" \
               "<LineString id=\"2\"> \n" \
               "<coordinates> \n"

    # List of co-ordinates read from kml file
    for points in co_ordinates:
        writeStr += str(points[0]) + "," + str(points[1]) + "," + str(points[2]) + "\n"

    writeStr += "</coordinates> \n" \
                "</LineString> \n" \
                "</Placemark> \n" \

    # Footer of Document
    endPart = "</Document> \n" \
                "</kml>"

    # Wriring a kml file without the placemarks for stop, traffic and turns
    fileOpen = open(fileName + "_Group3.kml", "w")
    fileOpen.write(writeStr + endPart)
    fileOpen.close()


    # Co-ordinates of points where we predict the stop signs
    for points in stopSign:
        writeStr += "<Placemark> \n" \
                    "<description>Stop Light</description> \n" \
                    "<Point>\n" \
                    "<coordinates> \n" +\
                    str(points[0]) + "," + str(points[1]) + "," + str(points[2]) + "\n" + \
                    "</coordinates>\n" + \
                    "</Point>\n" + \
                    "</Placemark>\n"

    # Co-ordinates of points where we predict the traffic lights
    for points in trafficLight:
        writeStr += "<Placemark>\n" \
               "<description>Red PIN for A Stop</description>\n" \
               "<Style id=\"normalPlacemark\">\n" \
               "<IconStyle>\n" \
               "<color>ff0000ff</color>\n" \
               "<Icon>\n" \
               "<href>http://maps.google.com/mapfiles/kml/paddle/1.png</href>\n" \
               "</Icon>\n" \
               "</IconStyle>\n" \
               "</Style>\n" \
               "<Point>\n" \
               "<coordinates>" + \
               str(points[0]) + "," + str(points[1]) + "," + str(points[2]) + "\n" + \
               "</coordinates>\n" \
               "</Point>\n" \
               "</Placemark>\n"

    # Co-ordinates of points where we predict the car is parked
    for points in parked:
        writeStr += "<Placemark>\n" \
                    "<description>Parked</description>" \
                    "<Style id=\"normalPlacemark\">\n" \
                    "<IconStyle>\n" \
                    "<color>ff008080</color>\n" \
                    "<Icon>\n" \
                    "<href>http://maps.google.com/mapfiles/kml/paddle/2.png</href>\n" \
                    "</Icon>\n" \
                    "</IconStyle>\n" \
                    "</Style>\n" \
                    "<Point>\n" \
                    "<coordinates>" + \
                    str(points[0]) + "," + str(points[1]) + "," + str(points[2]) + "\n" + \
                    "</coordinates>\n" \
                    "</Point>\n" \
                    "</Placemark>\n"

    # Co-ordinates of points where we predict car took a turn
    for points in turn:
        writeStr += "<Placemark>\n" \
                    "<description>Turns</description>" \
                    "<Style id=\"normalPlacemark\">\n" \
                    "<IconStyle>\n" \
                    "<color>f0C0B0B0</color>\n" \
                    "<Icon>\n" \
                    "<href>http://maps.google.com/mapfiles/kml/paddle/3.png</href>\n" \
                    "</Icon>\n" \
                    "</IconStyle>\n" \
                    "</Style>\n" \
                    "<Point>\n" \
                    "<coordinates>" + \
                    str(points[0]) + "," + str(points[1]) + "," + str(points[2]) + "\n" + \
                    "</coordinates>\n" \
                    "</Point>\n" \
                    "</Placemark>\n"

    # Saving the hazard file with markers for stop sign, traffic lights and turns
    fileOpen = open(fileName + "_HazardFile_Group3.kml", "w")
    fileOpen.write(writeStr + endPart)
    fileOpen.close()


def calculateCost(startTime, endTime, maxSpeed, totalNumberStops):
    """
    This is the method where we calculate the cost.
    The overall travel time is divided by 30 minutes i.e. average time of a journey
    Maximum speed in the journey is divided by 52.13 knot i.e. average spped in a journey
    Total number of times a car is stopped or took a turn is divided by 5 i.e. average time the situation arises in a
    journy
    :param startTime: Journey start time
    :param endTime: Journey end time
    :param maxSpeed: Maximum spped in knots of the journy
    :param totalNumberStops: Total time the car camw to a halt or took a turn,
    :return:
    """
    time = 30 * 60
    speedInKnot = 52.13
    startTime = getSeconds(startTime)
    endTime = getSeconds(endTime)
    averageStops = 5

    return ((endTime - startTime) / time) + 0.1 * (maxSpeed / speedInKnot) + (totalNumberStops / averageStops)

def main():
    """
    This is the main function where the gps files are read and then all the other method calls are made.
    There are 3 ways in which source gps file names can be provided.
    1-> Comma separated values. Pass all the file names separated by a comma
    2-> File names separated by a space between them
    3-> If no names are provided, default files are considered.
    :return:
    """
    fileName = []
    if len(sys.argv) == 2:
        # If file names are provided by comma separated
        fileName.extend(sys.argv[1].split(','))
    elif len(sys.argv) > 2:
        # If file names are provided by space separated
        for eachFile in range(1, len(sys.argv)):
            fileName.append(sys.argv[eachFile])
    else:
        # Default file names
        fileName = ['2019_10_05__210421_gps_file.txt', '2019_10_08__210327_gps_file.txt', '2019_03_12__1423_30.txt'
                    , '2019_03_13__2033_30.txt', '2019_03_20__2227_30.txt']

    costFunctions = [] # A list to store cost functions of every input file
    for files in fileName:
        with open(files.strip()) as gps_File:
            co_ordinates = []  # long, lat, speed in knots
            co_ordinates_list = []  # list of co_ordinates
            for gps_Data in gps_File:
                line = gps_Data.split(",")
                line[len(line) - 1] = line[len(line) - 1][:-1]
                if line[0] == "$GPRMC":
                    if len(line) == 13: # Making sure there is no garbage data
                        if line[3] != "" or line[5] != "": # Making sure co-ordinates are present
                            co_ordinatesValue = get_coordinates(line)
                            co_ordinates_list.append(co_ordinatesValue)
                            co_ordinates.append(tuple(co_ordinatesValue[:len(co_ordinatesValue)]))

        stopSign, trafficLight, parked, turn, maxSpeed = get_Stop_Signs(co_ordinates_list)
        costFunctions.append(calculateCost(co_ordinates_list[0][3],
                                           co_ordinates_list[len(co_ordinates_list) - 1][3], maxSpeed,
                                           len(stopSign) + len(trafficLight) + len(parked)))
        outputKML(co_ordinates, stopSign, trafficLight, parked, turn, files.split(".")[0])
    print("File %s provides the best path" %fileName[costFunctions.index(min(costFunctions))])


if __name__ == "__main__":
    main()