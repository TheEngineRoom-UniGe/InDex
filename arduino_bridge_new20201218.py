import rospy, tf, struct, socket, math
#import tf2_geometry_msgs
from sensor_msgs.msg import Imu

from geometry_msgs.msg import Quaternion, TransformStamped, Vector3Stamped, PoseStamped


UDP_IP = "130.251.13.113"#"192.168.43.94" #
UDP_PORT = 2390
debugPort = 2490;

names = ["thumb_1","thumb_2","index_1","index_2","middle_finger_1","middle_finger_2","6","7","8","9","10","11","12","13","14","ring_finger_1","ring_finger_2","pinkie_1","pinkie_2","back","wrist","hand2"]
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
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

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
	print ("listener initialized")
	while not rospy.is_shutdown():
		data = sock.recv(29)# (29) # buffer size is 1024 bytes
                #print(plane_angle(plane[19][0],plane[19][1],plane[19][2],plane[2][0],plane[2][1],plane[2][2] ))
		print("------------")
		if not data:
			print("no data")
		else:
			#print ("data received")
			#ID = ord(data[28])
			ID = data[28]
			msg = Imu()
			msg.header.stamp = rospy.Time.now()
			msg.orientation.x = struct.unpack('<f', data[0:4])[0]
			msg.orientation.y = struct.unpack('<f', data[4:8])[0]
			msg.orientation.z = struct.unpack('<f', data[8:12])[0]
			msg.orientation.w = struct.unpack('<f', data[12:16])[0]
			msg.header.frame_id='base_link'
			print_to_socket(msg)
			print ( "ID " , ID);
			msg.linear_acceleration.x = struct.unpack('<h', data[16:18])[0]
			msg.linear_acceleration.y = struct.unpack('<h', data[18:20])[0]
			msg.linear_acceleration.z = struct.unpack('<h', data[20:22])[0]
			#print ( "acc " , msg.linear_acceleration.x, );
			msg.angular_velocity.x = struct.unpack('<h', data[22:24])[0]
			msg.angular_velocity.y = struct.unpack('<h', data[24:26])[0]
			msg.angular_velocity.z = struct.unpack('<h', data[26:28])[0]      
			#print ( "av " , msg.angular_velocity.x, );
			#t.transform.rotation.x  =  msg.orientation.x 
			#t.transform.rotation.y  =  msg.orientation.y
			#t.transform.rotation.z  =  msg.orientation.z
			#t.transform.rotation.w  =  msg.orientation.w
			msg2 = PoseStamped(); 
			msg2.header.frame_id='base_link'
			msg2.header.stamp = msg.header.stamp;
			msg2.pose.orientation = msg.orientation; 
			msg2.pose.position.x = 0 ; 
			msg2.pose.position.y = 0; 
			msg2.pose.position.z = 0; 

#			vt = tf2_geometry_msgs.do_transform_vector3(v,t)    
#			plane[ID][0] = vt.vector.x 
#			plane[ID][1] = vt.vector.y
#			plane[ID][2] = vt.vector.z

#			if not(flag[ID]):
#				flag[ID] = True
#				euler = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y,msg.orientation.z,msg.orientation.w])
#				eulers[ID] = -euler[2]
#				br.sendTransform((x[ID], y[ID], 0.0), (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), rospy.Time.now(), names[ID]+"_regular", "root")
#				br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, eulers[ID]), rospy.Time.now(), names[ID]+"_rectified", names[ID]+"_regular")
#				if ID == 19:
#					br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), names[ID]+"_rotated", names[ID]+"_rectified")
#					quaternions[ID][0] = msg.orientation.x
#					quaternions[ID][1] = msg.orientation.y
#					quaternions[ID][2] = msg.orientation.z
#					quaternions[ID][3] = msg.orientation.w
#			else:
			
#				br.sendTransform((x[ID], y[ID], 0.0), (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), rospy.Time.now(), names[ID]+"_now", "root")
#				br.sendTransform((x[ID], y[ID], 0.0), (quaternions[ID][0], quaternions[ID][1], quaternions[ID][2], quaternions[ID][3]), rospy.Time.now(), names[ID]+"_regular", "root")
#				br.sendTransform((0.0, 0.0, 0.0),  tf.transformations.quaternion_from_euler(0, 0, eulers[ID]), rospy.Time.now(), names[ID]+"_rectified", names[ID]+"_regular")
#				diff_q = tf.transformations.quaternion_multiply(quaternions[ID][:], [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
#			try:
#				(trans,rot) = listener.lookupTransform(names[ID]+"_regular", names[ID]+"_now", rospy.Time(0))
#				if ID == 19:
#					br.sendTransform((0.0, 0.0, 0.0), (rot[0], rot[1], rot[2], rot[3]), rospy.Time.now(), names[ID], names[ID]+"_rectified")
#				else:
#					br.sendTransform((0.0, 0.0, 0.0), (rot[0], rot[1], rot[2], rot[3]), rospy.Time.now(), names[ID], names[ID]+"_rectified")
			    #msg.position[name.index(joint_list[i])] = euler[0]
			    #print(str(transform_list[i]) + " " + str(euler))
#			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#			    continue

			#print(diff_q)
			#diff_q = old_Q - msg.orientation

			#print(plane)
			#br.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), "root_rotated", "root")
		#	print ("msg received");
		#	pub = rospy.Publisher(names[ID], Imu, queue_size=0)
		#	pub.publish(msg)
			pub2 = rospy.Publisher(names[ID]+"Pose", PoseStamped, queue_size=0)
			pub2.publish (msg2)


def print_to_socket(msg): 
	
	UDP_IP_ = "127.0.0.1"
	UDP_PORT_ = 5005
	MESSAGE = (f'{"w"}{"{:.2f}".format(msg.orientation.x)}{"w"}{"a"}{"{:.2f}".format(msg.orientation.y)}{"a"}{"b"}{"{:.2f}".format(msg.orientation.z)}{"b"}{"c"}{"{:.2f}".format(msg.orientation.w)}{"c"}')
	print(msg.header.stamp)
	#print("UDP target IP: %s" % UDP_IP)
#	print("UDP target port: %s" % UDP_PORT)
#	print("message: %s" % MESSAGE)
	 
	sock = socket.socket(socket.AF_INET, # Internet
		              socket.SOCK_DGRAM) # UDP
	try:
		sock.sendto(bytes(MESSAGE,'utf-8'), (UDP_IP_, UDP_PORT_))
	except ValueError:
		print (" error sock");
		
def main():
    rospy.init_node('arduino_bridge', disable_signals=True)
    #print ("main called")
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

