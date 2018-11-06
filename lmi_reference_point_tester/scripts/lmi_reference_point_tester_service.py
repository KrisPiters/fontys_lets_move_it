#!/usr/bin/env python
import rospy
import tf
from lmi_reference_point_tester.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, \
                              Pose, \
                              Point, \
                              Quaternion, \
                              TwistWithCovariance, \
                              Twist, \
                              Vector3
from std_msgs.msg import Header
from copy import deepcopy

class reference_point_core:
 
  def __init__(self,
      map_frame_id,
      robot_frame_id,
      odom_topic_name,
      transformed_odom_topic_name, 
      transformed_robot_pose_topic_name, 
      transformed_robot_pose_stamped_topic_name):
    
    self._map_frame_id = map_frame_id
    self._robot_frame_id = robot_frame_id
    self._odom_topic_name = odom_topic_name
    self._transformed_odom_topic_name = transformed_odom_topic_name
    self._transformed_robot_pose_topic_name = transformed_robot_pose_topic_name
    self._transformed_robot_pose_stamped_topic_name = transformed_robot_pose_stamped_topic_name 
    
    self._tf_listener = tf.TransformListener()
    self._tf_broadcaster = tf.TransformBroadcaster()
    self._reference_point_frame_id = ""
    self._create_ref_point_service = rospy.Service('~create_ref_point', CreateRefPoint, self.create_ref_point_handler)
    self._odom_sub = rospy.Subscriber(self._odom_topic_name, Odometry, self.odometry_callback)
    self._transformed_odom_pub = rospy.Publisher(self._transformed_odom_topic_name, Odometry, queue_size= 100)
    self._transformed_robot_pose_pub = rospy.Publisher(self._transformed_robot_pose_topic_name, Pose, queue_size= 100)
    self._transformed_robot_pose_stamped_pub = rospy.Publisher(self._transformed_robot_pose_stamped_topic_name, PoseStamped, queue_size= 100)
    self._latest_robot_odom = Odometry()

    self._ref_point_to_map_transform_translation = None
    self._ref_point_to_map_transform_rotation = None

    rospy.loginfo("Reference Point Core initialized")
    rospy.loginfo("map_frame_id: %s", self._map_frame_id)
    rospy.loginfo("robot_frame_id: %s", self._robot_frame_id)
    rospy.loginfo("odom_topic_in: %s", self._odom_topic_name)
    rospy.loginfo("transformed_odom_topic_out: %s", transformed_odom_topic_name)
    rospy.loginfo("transformed_robot_pose_topic_out: %s", self._transformed_robot_pose_topic_name)
    rospy.loginfo("transformed_robot_pose_stamped_topic_out: %s", self._transformed_robot_pose_stamped_topic_name)

  
  def create_ref_point_handler(self, req):
    rospy.loginfo("[set_ref_point] Service received request to set reference point with name: %s"%(req.name))
    
    current_time = rospy.Time(0) #rospy.Time.now()

    if (self._tf_listener.canTransform(self._robot_frame_id, self._map_frame_id, current_time)):
      
      (self._ref_point_to_map_transform_translation, self._ref_point_to_map_transform_rotation) = \
                                        self._tf_listener.lookupTransform( self._robot_frame_id, 
                                                                           self._map_frame_id,  
                                                                           current_time)
      
      rospy.loginfo("Transform found and saved: %s, %s", self._ref_point_to_map_transform_translation, self._ref_point_to_map_transform_rotation)
      
      self._reference_point_frame_id = req.name
      
      self.send_reference_frame_transform()

      return CreateRefPointResponse(1)

    rospy.loginfo("Unable to set reference point; No transform found from %s to %s",self._robot_frame_id, self._map_frame_id)
    return CreateRefPointResponse(0)

  def odometry_callback(self, odom_data):
    
    ref_id = self._reference_point_frame_id
    rob_id = self._robot_frame_id

    # using Time(0) because for unkown reasons using the stamp from odom_data fails
    time = rospy.Time(0)

    #rospy.loginfo("odom time: %s rostime.now: %s", odom_data.header.stamp, rospy.Time.now())

    if (ref_id is not ""):
      if (self._tf_listener.canTransform(ref_id, rob_id, time)):#odom_data.header.stamp)):
        
        # Create posestamped from odom pose
        ps = PoseStamped()
        ps.header.frame_id = rob_id
        ps.header.stamp = time #odom_data.header.stamp
        ps.header.seq = odom_data.header.seq

        # Transform the posestamped to reference_frame
        tfp = self._tf_listener.transformPose(ref_id, ps)

        # Create odom message with transformed pose
        tf_odom = Odometry()
        tf_odom.header = odom_data.header
        tf_odom.header.frame_id = ref_id
        tf_odom.child_frame_id = rob_id
        
        tf_odom.pose = odom_data.pose
        
        # replace original pose with transformed pose form pose stamped
        tf_odom.pose.pose = tfp.pose
        tf_odom.twist = odom_data.twist

        # publish transformed data
        self._transformed_odom_pub.publish(tf_odom)
      
      else:
        rospy.loginfo("Unable to transform odometry: no transform from %s to %s at time %s;", self._reference_point_frame_id, self._robot_frame_id, odom_data.header.stamp)


          
  def send_reference_frame_transform(self):

    if (self._reference_point_frame_id is not ""):
      self._tf_broadcaster.sendTransform( self._ref_point_to_map_transform_translation, 
                                          self._ref_point_to_map_transform_rotation, 
                                          rospy.Time.now(), 
                                          self._map_frame_id, 
                                          self._reference_point_frame_id)

  # Currently not working
  '''
  def publish_transformed_odometry(self):
    
    if not self._transformed_odom_pub.get_num_connections > 0:
      return
  
    if (self._reference_point_frame_id is not "" and self._latest_robot_odom is not None):
     
      if (self._tf_listener.canTransform(self._reference_point_frame_id, self._robot_frame_id, rospy.Time.now())):
        try:

          transformed_odom_ = Odometry( header = self._latest_robot_odom.header,
                                        child_frame_id = self._latest_robot_odom.child_frame_id,
                                        pose = self._latest_robot_odom.pose,
                                        twist = self._latest_robot_odom.twist)

          (transformed_odom_.pose.pose.position, transformed_odom_.pose.pose.orientation) = \
            self._tf_listener.lookupTransform( self._reference_point_frame_id, 
                                               self._robot_frame_id,  
                                               rospy.Time.now())
          
          #transformed_odom_.pose.pose = self._tf_listener.transformPose(  self._reference_point_frame_id,
          #                                                                PoseStamped(header = odom_data.header,
          #                                                                            pose = odom_data.pose.pose))
          transformed_odom_.header.frame_id = self._reference_point_frame_id
          transformed_odom_.header.stamp = rospy.Time.now()
          rospy.loginfo("Publishing transformed odometry.")
          #self._transformed_odom_pub.publish(transformed_odom_)
          self._transformed_odom_pub.publish(self._latest_robot_odom)
          rospy.loginfo("Published transformed odometry. rostime: %s", rospy.Time.now())
          
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          rospy.loginfo("Unable to transform odom pose to reference frame: tf exception.")
          pass
      else:
        rospy.loginfo("Unable to transform odom pose to reference frame: No transform available.")
    else:
      rospy.loginfo("Unable to transform odom pose to reference frame: Reference frame_id is empty")
  '''

  def publish_transformed_robot_poses(self):
    pose = Pose()
    pose_stamped = PoseStamped()
    time = rospy.Time(0)

    if self._reference_point_frame_id != "" and \
        self._tf_listener.canTransform(self._reference_point_frame_id, self._robot_frame_id, time):
      try:
        (translation, orientation) = self._tf_listener.lookupTransform( self._reference_point_frame_id, 
                                                                        self._robot_frame_id,  
                                                                        time)

        pose.position = Point(  translation[0], 
                                translation[1], 
                                translation[2])

        pose.orientation = Quaternion(  orientation[0],
                                        orientation[1],
                                        orientation[2],
                                        orientation[3])

        pose_stamped.header.stamp = time
        pose_stamped.header.frame_id = self._reference_point_frame_id
        pose_stamped.pose = pose

        self._transformed_robot_pose_pub.publish(pose)
        self._transformed_robot_pose_stamped_pub.publish(pose_stamped)

      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("Unable to transform odom pose to reference frame: tf exception.")
        pass

  def loop(self):
    self.send_reference_frame_transform()
    #self.publish_transformed_odometry()
    self.publish_transformed_robot_poses()
  

if __name__ == "__main__":
  rospy.init_node('reference_point_provider')

  map_frame_id = rospy.get_param('~map_frame_id','map')
  robot_frame_id = rospy.get_param('~robot_frame_id', 'base_link')
  odom_topic_name = rospy.get_param('~odom_in_topic','odom')
  transformed_odom_topic_name = rospy.get_param('~transformed_odom_topic','~transformed_odom')
  transformed_robot_pose_topic_name = rospy.get_param('~transformed_robot_pose_topic','~transformed_robot_pose')
  transformed_robot_pose_stamped_topic_name = rospy.get_param('~transformed_robot_pose_stamped_topic','~transformed_robot_pose_stamped')
  
  rate = rospy.Rate(rospy.get_param('~rate', 10))
  
  reference_point_core_ = reference_point_core(
    map_frame_id,
    robot_frame_id,
    odom_topic_name,
    transformed_odom_topic_name,
    transformed_robot_pose_topic_name,
    transformed_robot_pose_stamped_topic_name)

  rospy.sleep(5)
  
  while not rospy.is_shutdown():
    reference_point_core_.loop()
    rate.sleep()
    
