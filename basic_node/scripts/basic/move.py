# Radar elements
import math
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud

label= ['Undefine']

class Point():
    OBSTACLE= 0
    NEGATIF= 1
    UNDEFINE= 2
    SIZE= 3
    labels= ['Obstacles', 'Negatif', 'Undefine']

    # Construction:
    def __init__(self, x= 0.0, y= 0.0, label= OBSTACLE):
        self.x= x
        self.y= y
        self.label= label

    def fromRadialCoordinates( self, anAngle, aDistance ):
        self.x= (math.cos(anAngle) * aDistance)
        self.y= (math.sin(anAngle) * aDistance)
        return self

    # Accessor:
    def coordinates(self):
        return self.x, self.y
    
    def distance(self):
        return math.sqrt( (self.x*self.x) + (self.y*self.y) )

    def distanceSquare(self):
        return (self.x*self.x) + (self.y*self.y)

    def coordinates(self):
        return self.x, self.y

class Radar():
    # Construction:
    def __init__(self):
        self._points= []

    def clear(self):
        self._points= []
    
    def append(self, aPoint):
        dist2= aPoint.distanceSquare()
        if 0.01 < dist2 and dist2 < 100.0 :
            self._points.append( aPoint )
        return self
    
    # Accessors:
    def points(self):
        return  self._points
    
    def coordinates(self):
        return [ p.coordinates() for p in self._points ]
    
    # Printing:
    def printSample( self, printFct=print ):
        sample= [ [ round(p.x, 2), round(p.y, 2) ] for p in self._points[10:15] ]
        printFct( f"\npc({len(self._points)}) ...{sample}..." )

class RosRadar():

    # Construction:
    def __init__(self, rosNode):
        self._radar= Radar()
        self._pc_publisher= None
        self.rosNode= rosNode

    # Initialize Processes:
    def initScanProcess( self, scanTopic= 'scan', pubTopic= 'obstacles' ):
        self._pc_publisher= self.rosNode.create_publisher( PointCloud, pubTopic, 10)
        self.rosNode.create_subscription( LaserScan, scanTopic, self.scan_callback, 10)

    # Callbacks:
    def scan_callback(self, scanMsg):
        self._radar.clear()
        angle= scanMsg.angle_min
        for distance in scanMsg.ranges :
            self._radar.append( Point( label=Point.OBSTACLE ).fromRadialCoordinates( angle, distance ) )
            angle+= scanMsg.angle_increment
        self.publish_pointcloud( scanMsg.header )
    
    def publish_pointcloud(self, header):
        self.obstacles= PointCloud()
        self.obstacles.header= header
        zero= (float)(0)
        for x, y in self._radar.coordinates() :
            aPoint= Point32()
            aPoint.x= (float)(x)
            aPoint.y= (float)(y)
            aPoint.z= zero
            self.obstacles.points.append( aPoint )
        
        self._radar.printSample( printFct= self.rosNode.get_logger().info )
        self._pc_publisher.publish( self.obstacles )

    # Accessors: 
    def points(self):
        return self._radar.points()
