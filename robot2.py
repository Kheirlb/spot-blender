import bpy
import mathutils
import math

D = bpy.data
M = mathutils.Matrix

class robot:
    """A robot class for interacting with a spot simulator in Blender"""
    # robot geometry for frame assignment
    # body geometry
    bodyXDim = 6
    bodyYDim = 1.6
    bodyZDim = 2
    
    # leg and foot values
    hipLength = 0.75
    legWidth = 0.5
    halfLegWidth = legWidth/2
    power_off_feet = 1.25
    feetOutNormal = halfLegWidth + hipLength + bodyYDim/2
    feetBack = 0.25
    footRadius = 0.25
    hipLinkLength = halfLegWidth + hipLength
    thighLength = 3
    legLength = 3
    
    # controls
    feetOutCurrent = feetOutNormal + power_off_feet
    bodyYaw = 0.0
    bodyRoll = 0.0
    bodyPitch = 0.0
    bodyX = 0
    bodyY = 0
    bodyZ = 1
    footControlX = 0
    footControlY = 0
    footControlZ = 0
    eulerMode = 'ZXY'
    footprintFrame = mathutils.Matrix.Identity(4)
    footFlag = 0
    
    def __init__(self, name):
        # simply assigns a name
        self.name = name

    def create_robot(self):
        # using default parameters, rearranges frames to standard position
        self.transform_robot()

    def power_on(self):
        self.crouch()

    # all following functions adjust position and orientation of robot
    def crouch(self):
        self.feetOutCurrent = self.feetOutNormal
        self.transform_body(0.0, 0.0, 0.0, 0, 0, 2)
        self.transform_robot()

    def stand_tall(self):
        self.feetOutCurrent = self.feetOutNormal
        self.transform_body(0.0, 0.0, 0.0, 0, 0, 4)
        self.transform_robot()
        
    def pose1(self):
        self.feetOutCurrent = self.feetOutNormal
        self.transform_body(0.4, 0.0, 0.0, 0, 0, 4)
        self.transform_robot()    

    def pose2(self):
        self.feetOutCurrent = self.feetOutNormal
        self.transform_body(0.4, 0.1, 0.4, 0, 0, 4)
        self.transform_robot()    

    def power_off(self):
        self.feetOutCurrent = self.feetOutNormal + self.power_off_feet
        self.transform_body(0.0, 0.0, 0.0, 0, 0, 1)
        self.transform_robot()    

    def transform_body(self, yaw=None, roll=None, pitch=None, X=None, Y=None, Z=None):
        """Transforms Body Frame"""
        # this function demonstrates use of default arguments and None type checking
        if yaw is None:
            yaw = self.bodyYaw
        self.bodyYaw = yaw
        if roll is None:
            roll = self.bodyRoll
        self.bodyRoll = roll
        if pitch is None:
            pitch = self.bodyPitch
        self.bodyPitch = pitch
        if X is None:
            X = self.bodyX
        self.bodyX = X
        if Y is None:
            Y = self.bodyY
        self.bodyY = Y
        if Z is None:
            Z = self.bodyZ
        self.bodyZ = Z
        self.transform_robot()

    def move_foot(self, legNum, yaw=0.0, roll=0.0, pitch=0.0, X=0, Y=0, Z=0):
        """shifts foot position relative to current position"""
        self.footControlX = X
        self.footControlY = Y
        self.footControlZ = Z
        self.footFlag = 1
        self.transform_leg(legNum)

    def move(self, yaw=0.0, roll=0.0, pitch=0.0, X=0, Y=0, Z=0):
        """moves footprint frame"""
        new = self.createTransform((yaw, roll, pitch),(X, Y, Z))
        self.footprintFrame = new
        self.transform_robot()
        
    # functions below this point are considered "protected"
    def setEulerMode(self, order):
        """Sets Eular Mode for All Controls"""
        self.eulerMode = order

    def transform_robot(self):
        """main function for generating transformation matrices and moving robot"""
        # transform body
        self.body_R_footprint = self.createTransform(\
        (self.bodyYaw, self.bodyRoll, self.bodyPitch), (self.bodyX, self.bodyY, self.bodyZ))
        self.placeObj('bodyF', self.body_R_footprint)   
        # tranform legs     
        self.transform_leg(1)
        self.transform_leg(2)
        self.transform_leg(3)
        self.transform_leg(4)

    def transform_leg(self, legNum):
        """Transforms the frames of a specific leg"""
        # 1 = front left
        # 2 = front right
        # 3 = back left
        # 4 = back right
        
        # for specific legs, use specific frame loc/rot
        if legNum == 1 or legNum == 3:
            jointALocAtt = [math.pi/2, 0.0, math.pi/2]
            jointALocPos = [self.bodyXDim/2, self.bodyYDim/2, 0]
            feetOutLegNum = self.feetOutCurrent
            zAxisHip = -math.pi/2
        if legNum == 2 or legNum == 4:
            jointALocAtt = [-math.pi/2, 0.0, math.pi/2]
            jointALocPos = [self.bodyXDim/2, -self.bodyYDim/2, 0]
            feetOutLegNum = -self.feetOutCurrent
            zAxisHip = math.pi/2
        if legNum == 3 or legNum == 4:
            jointALocPos[0] = -jointALocPos[0]
            
        # transform joint A Loc
        jointALoc_R_body = self.createTransform(jointALocAtt, jointALocPos)
        jointALoc_R_footprint = self.body_R_footprint @ jointALoc_R_body
        self.placeObj('jointALocF' + str(legNum), jointALoc_R_footprint)
        
        # transform foot Control
        # placing foot off to the side
        footControlPos = [jointALocPos[0]-self.feetBack, feetOutLegNum, self.footRadius];
        # check if special foot movement desired
        if self.footFlag == 1:
            footControlPos[0] = footControlPos[0] + self.footControlX
            footControlPos[1] = footControlPos[1] + self.footControlY
            footControlPos[2] = footControlPos[2] + self.footControlZ
            self.footFlag = 0
        footControl_R_footprint = self.createTransform((0,0,0), footControlPos)
        self.placeObj('footControlF' + str(legNum), footControl_R_footprint)
        
        # calculate theta 0 angle or jointA angle
        footControl_R_jointALoc = M.inverted(M.inverted(footControl_R_footprint) @ jointALoc_R_footprint)
        #print(footControl_R_jointALoc)
        temp_theta_0 = math.atan(footControl_R_jointALoc[0][3]/footControl_R_jointALoc[1][3])
        #print(temp_theta_0)
        temp_dis = footControl_R_jointALoc[0][3]/math.sin(temp_theta_0)
        #print(temp_dis)
        theta_0 = temp_theta_0 - math.asin(self.hipLinkLength/temp_dis)
        #print(theta_0)
        
        # transform joint A Rot
        jointARot_R_self = self.createTransform((-theta_0, 0.0, 0.0), (0, 0, 0))
        jointARot_R_footprint = jointALoc_R_footprint @ jointARot_R_self
        self.placeObj('jointARotF' + str(legNum), jointARot_R_footprint)
        
        # transform hip Loc
        hipLoc_R_self = self.createTransform((0.0, 0.0, zAxisHip), \
        (self.hipLinkLength, 0, 0))
        hipLoc_R_footprint = jointARot_R_footprint @ hipLoc_R_self
        self.placeObj('hipLocF' + str(legNum), hipLoc_R_footprint)
        
        # IK for theta 1 (hip) and theta 2 (knee)
        footControl_R_hipLoc = M.inverted(M.inverted(footControl_R_footprint) @ hipLoc_R_footprint)
        x = footControl_R_hipLoc[0][3];
        y = footControl_R_hipLoc[1][3];
        l1 = self.thighLength
        l2 = self.legLength
        ct2 = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2);
        st2 = math.sqrt(1-ct2**2);
        theta2 = math.atan2(st2,ct2);
        k1 = l1 + l2*math.cos(theta2);
        k2 = l2*math.sin(theta2);
        theta1 = math.atan2(y,x) - math.atan2(k2,k1);
        
        # transform hip Rot
        hipRot_R_self = self.createTransform((theta1, 0.0, 0.0), (0, 0, 0))
        hipRot_R_footprint = hipLoc_R_footprint @ hipRot_R_self
        self.placeObj('hipRotF' + str(legNum), hipRot_R_footprint)

        # transform knee Loc
        kneeLoc_R_self = self.createTransform((0.0, 0.0, 0.0), (self.thighLength, 0, 0))
        kneeLoc_R_footprint = hipRot_R_footprint @ kneeLoc_R_self
        self.placeObj('kneeLocF' + str(legNum), kneeLoc_R_footprint)
        
        # transform knee Rot
        kneeRot_R_self = self.createTransform((theta2, 0.0, 0.0), (0, 0, 0))
        kneeRot_R_footprint = kneeLoc_R_footprint @ kneeRot_R_self
        self.placeObj('kneeRotF' + str(legNum), kneeRot_R_footprint)

    def createTransform(self, angles, pos):
        """Creates a 4x4 transformation matrix from eular angles and position vector"""
        tempAngles = (angles[1], angles[2], angles[0])
        eul = mathutils.Euler(tempAngles, self.eulerMode)
        mat_rot = eul.to_matrix()
        mat_loc = mathutils.Matrix.Translation(pos)
        mat = mat_loc @ mat_rot.to_4x4()
        return mat
    
    def placeObj(self, objStr, frame_R_footprint):
        """places object in respect to world frame"""
        # grab frame
        obj = D.objects[objStr]
        frame_R_world = self.footprintFrame @ frame_R_footprint
        obj.location = frame_R_world.col[3][0:3]
        obj.rotation_mode = self.eulerMode
        obj.rotation_euler = frame_R_world.to_euler(obj.rotation_mode)

# main function below
print('\n---Loaded Spot Robot Simulator---\n')

spot = robot('B99')
spot.create_robot()

#testing code
#spot.power_on()
#spot.stand_tall()
#spot.move_footprint_frame(yaw=math.pi/4, X=2)
#spot.transform_body(bodyYaw=0.4)
#spot.pose1()
#spot.transform_body(bodyPitch=0.2)
#spot.move_foot(1, X=1)
#spot.stand_tall()
#spot.power_off()

del spot