#!/usr/bin/python2.7
import rospy, tf, struct, socket, math
import tf2_geometry_msgs
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3Stamped


UDP_IP = "130.251.13.72"
UDP_PORT = 2390

names = ["thumb_1","thumb_2","index_1","index_2","middle_finger_1","middle_finger_2","6","7","8","9","10","11","12","13","14","ring_finger_1","ring_finger_2","pinkie_1","pinkie_2","back","wrist"]
#names = ["thumb_distal","thumb_meta","index_distal","index_meta","middle_distal","middle_meta","6","7","8","9","10","11","12","13","14","ring_distal","ring_meta","pinky_distal","pinky_meta","base","wrist"]

x = [0, 0.5, 1, 1.5, 1, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1.5, 1, 1.5, 0, -5]
y = [1, 1, 0, 0, 0.5, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1.5, 1.5, 0, 0]

flag = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]

offset = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

plane = [[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]]
quaternions = [[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0]]
eulers = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


v = Vector3Stamped()
v.vector.x = 0
v.vector.y = 0
v.vector.z = 1

t = TransformStamped()

def plane_angle(a1, b1, c1, a2, b2, c2):
    divider = math.sqrt(pow(a1,2)+pow(b1,2)+pow(c1,2))*math.sqrt(pow(a2,2)+pow(b2,2)+pow(c2,2))
    if divider == 0:
        return 0
    else:
        result = abs(a1*a1 + b1*b2 + c1*c2)/(divider)
        if result > 1:
            return math.acos(1)
        else:
            return math.acos(result)


def listener():
	sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)                      
	sock.bind((UDP_IP, UDP_PORT)) 
	br = tf.TransformBroadcaster()
        listener = tf.TransformListener()
	
	
	while not rospy.is_shutdown():
		data = sock.recv(29) # buffer size is 1024 bytes
                #print(plane_angle(plane[19][0],plane[19][1],plane[19][2],plane[2][0],plane[2][1],plane[2][2] ))
	       # print("------------")
		if not data:
        		print("no data")
       		else:
                    
         	    ID = ord(data[28])
                    msg = Imu()
                    msg.header.stamp = rospy.Time.now()
                    msg.orientation.x = struct.unpack('<f', data[0:4])[0]
                    msg.orientation.y = struct.unpack('<f', data[4:8])[0]
                    msg.orientation.z = struct.unpack('<f', data[8:12])[0]
                    msg.orientation.w = struct.unpack('<f', data[12:16])[0]

                    msg.linear_acceleration.x = struct.unpack('<h', data[16:18])[0]
                    msg.linear_acceleration.y = struct.unpack('<h', data[18:20])[0]
                    msg.linear_acceleration.z = struct.unpack('<h', data[20:22])[0]

                    msg.angular_velocity.x = struct.unpack('<h', data[22:24])[0]
                    msg.angular_velocity.y = struct.unpack('<h', data[24:26])[0]
                    msg.angular_velocity.z = struct.unpack('<h', data[26:28])[0]      

                    t.transform.rotation.x  =  msg.orientation.x 
                    t.transform.rotation.y  =  msg.orientation.y
                    t.transform.rotation.z  =  msg.orientation.z
                    t.transform.rotation.w  =  msg.orientation.w

                    vt = tf2_geometry_msgs.do_transform_vector3(v,t)    
                    plane[ID][0] = vt.vector.x 
                    plane[ID][1] = vt.vector.y
                    plane[ID][2] = vt.vector.z

                    if not(flag[ID]):
                        flag[ID] = True
                        euler = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
                        eulers[ID] = -euler[2]
                        br.sendTransform((x[ID], y[ID], 0.0), (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), rospy.Time.now(), names[ID]+"_regular", "root")
                        br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, eulers[ID]), rospy.Time.now(), names[ID]+"_rectified", names[ID]+"_regular")
                        if ID == 19:
                            br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), names[ID]+"_rotated", names[ID]+"_rectified")
                        quaternions[ID][0] = msg.orientation.x
                        quaternions[ID][1] = msg.orientation.y
                        quaternions[ID][2] = msg.orientation.z
                        quaternions[ID][3] = msg.orientation.w
                    else:
                        #old_Q = Quaternion()
                        #old_Q.x = quaternions[ID][0]
                        #old_Q.y = quaternions[ID][1]
                        #old_Q.z = quaternions[ID][2]
                        #old_Q.w = quaternions[ID][3]
                        br.sendTransform((x[ID], y[ID], 0.0), (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), rospy.Time.now(), names[ID]+"_now", "root")
                        br.sendTransform((x[ID], y[ID], 0.0), (quaternions[ID][0], quaternions[ID][1], quaternions[ID][2], quaternions[ID][3]), rospy.Time.now(), names[ID]+"_regular", "root")
                        br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, eulers[ID]), rospy.Time.now(), names[ID]+"_rectified", names[ID]+"_regular")
                        diff_q = tf.transformations.quaternion_multiply(quaternions[ID][:], [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
                        try:
                            (trans,rot) = listener.lookupTransform(names[ID]+"_regular", names[ID]+"_now", rospy.Time(0))
                            if ID == 19:
                               # rot = tf.transformations.quaternion_multiply(rot, tf.transformations.quaternion_from_euler(0, 0, -1.57))
                               # br.sendTransform((0.0, 0.0, 0.0), (rot[0], rot[1], rot[2], rot[3]), rospy.Time.now(), names[ID]+"_rotated", names[ID]+"_rectified")
                               # br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, -1.57), rospy.Time.now(), names[ID], names[ID]+"_rotated")
                               # 
                                br.sendTransform((0.0, 0.0, 0.0), (rot[0], rot[1], rot[2], rot[3]), rospy.Time.now(), names[ID], names[ID]+"_rectified")
                            else:
                                br.sendTransform((0.0, 0.0, 0.0), (rot[0], rot[1], rot[2], rot[3]), rospy.Time.now(), names[ID], names[ID]+"_rectified")
                            #msg.position[name.index(joint_list[i])] = euler[0]
                            #print(str(transform_list[i]) + " " + str(euler))
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            continue

                        #print(diff_q)
                        #diff_q = old_Q - msg.orientation
                        
                    #print(plane)
                    #br.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), "root_rotated", "root")
                    '''
                    if not(flag[ID]):
                        flag[ID] = True
                        euler = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
                        offset[ID] = euler[0]
                    
                    euler = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
                    #print(euler)
                    if ID == 19:
                        br.sendTransform((x[ID], y[ID], 0.0), (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), rospy.Time.now(), names[ID], "root")
                        #br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, -1.57), rospy.Time.now(), names[ID]+"_rotated", names[ID])
                        br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, -euler[2]), rospy.Time.now(), names[ID]+"_rectified", names[ID])
                    else:
                        br.sendTransform((x[ID], y[ID], 0.0), (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), rospy.Time.now(), names[ID], "root")
                        br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, -euler[2]), rospy.Time.now(), names[ID]+"_rectified", names[ID])
                    
                    if ID == 19:
                        br.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), names[ID]+"_rectified", names[ID])
                    else:
                        br.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0, 0, -offset[ID]), rospy.Time.now(), names[ID]+"_rectified", names[ID])
                    '''
                    
                    pub = rospy.Publisher(names[ID], Imu, queue_size=0)
                    pub.publish(msg)

                    #print("received data from sensor: " + names[ID])
                    #print("orrientation -- x: " + str(msg.orientation.x) + " y: " + str(msg.orientation.y) + " z: " + str((msg.orientation.z) + " w: " + str(msg.orientation.w))
                    #print("accelerometer -- x: " + str(msg.linear_acceleration.x) + " y: " + str(msg.linear_acceleration.y) + " z: " + str(msg.linear_acceleration.z))
                    #print("gyroscope -- x: " + str(msg.angular_velocity.x) + " y: " + str(msg.angular_velocity.y) + " z: " + str(msg.angular_velocity.z))
        	
		
def main():
    rospy.init_node('arduino_bridge', disable_signals=True)
    listener()

    #rospy.Subscriber("/imu_data", Imu, callback)


    '''
    r = rospy.Rate(100)
    while True:
        try:
            r.sleep()
        except KeyboardInterrupt:            
            break'''
            

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

