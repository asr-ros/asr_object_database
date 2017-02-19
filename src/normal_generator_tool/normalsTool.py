#!/usr/bin/env python

'''
Copyright (C) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heizmann Heinrich, Heller Florian, Mehlhaus Jonas, Meissner Pascal, Schleicher Ralf, Stoeckle Patrick, Walter Milena

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
'''

import sys
import math
import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rosparam import upload_params
from yaml import load
import rospkg
import colorsys

def multiply_matrix(matrix_1,matrix_2):

    result_matrix = []
    if len(matrix_1[0]) != len(matrix_2):
        print "Invalid Matrix"
        return False
    for i in range(len(matrix_1)):
        line = []
        for j in range(len(matrix_2[0])):
            element = 0
            for k in range(len(matrix_1[0])):
                element = element + matrix_1[i][k] * matrix_2[k][j]
            line.append(element)
        result_matrix.append(line)
    return result_matrix

def matrix_vector_mult(matrix,vector):
    result_vect = []
    for mat in matrix:
        result_vect.append(mat[0]*vector[0]+mat[1]*vector[1]+mat[2]*vector[2])
    return result_vect


def get_rotation_matrix(angle_x, angle_y, angle_z):
    rotation_matrix_x = []
    rotation_matrix_x.append([1,0,0])
    rotation_matrix_x.append([0,math.cos(angle_x),-math.sin(angle_x)])
    rotation_matrix_x.append([0,math.sin(angle_x),math.cos(angle_x)])

    rotation_matrix_y = []
    rotation_matrix_y.append([math.cos(angle_y),0,math.sin(angle_y)])
    rotation_matrix_y.append([0,1,0])
    rotation_matrix_y.append([-math.sin(angle_y),0,math.cos(angle_y)])

    rotation_matrix_z = []
    rotation_matrix_z.append([math.cos(angle_z),-math.sin(angle_z),0])
    rotation_matrix_z.append([math.sin(angle_z),math.cos(angle_z),0])
    rotation_matrix_z.append([0,0,1])

    return multiply_matrix(rotation_matrix_z,multiply_matrix(rotation_matrix_y,rotation_matrix_x))

def xyz_to_sph(x, y, z):
    r = math.sqrt(x**2 + y**2 + z**2)
    elev = math.acos(z / r)
    az = math.atan2(y,x)
    return r, elev, az

def sph_to_xyz(r, elev, az):
    x = r * math.sin(elev) * math.cos(az)
    y = r * math.sin(elev) * math.sin(az)
    z = r * math.cos(elev)
    return x, y, z
    
def add_angles_to_sph(sph, d_elev, d_az):
    r = sph[0]
    elev = sph[1] + d_elev
    az = sph[2] + d_az
    return r, elev, az

def normalize_vector(vec):
    norm = math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
    if (norm != 0):
        return vec[0] / norm, vec[1] / norm, vec[2] / norm
    else:
        return vec

def round_vector_elements(vec):
    x = float("{0:.5f}".format(vec[0]))
    y = float("{0:.5f}".format(vec[1]))
    z = float("{0:.5f}".format(vec[2]))
    return x, y, z

if __name__ == "__main__":
    rospy.init_node('normal_generator_tool', anonymous=True)
    rospack = rospkg.RosPack()

    #Read the parameters from the configuration file
    f = open(rospack.get_path('asr_object_database') + '/src/normal_generator_tool/normalsToolParams.yaml', 'r')
    yamlfile = load(f)
    f.close()
    upload_params('/', yamlfile)

    marker_frame = rospy.get_param('marker_frame')
    pub_topic = rospy.get_param('pub_topic')
    output_rel_path = rospy.get_param('output_rel_path')
    base_vectors = rospy.get_param('base_vectors')
    pan_min = rospy.get_param('pan')[0]
    pan_max = rospy.get_param('pan')[1]
    tilt_min = rospy.get_param('tilt')[0]
    tilt_max = rospy.get_param('tilt')[1]
    d_x = rospy.get_param('d_pan')
    d_y = rospy.get_param('d_tilt')
    publish_rate = rospy.get_param('publish_rate')

    publisher = rospy.Publisher(pub_topic, MarkerArray, queue_size=10)

    #Use those Rotations to rotate the input vectors from the zx-plane to the xy-plane for the creation of the additional normal vectors
    # (to match the orientation of the object meshes)
    an_x = 90.0*math.pi/180
    rot_mat = get_rotation_matrix(an_x, 0.0, 0.0)
    rot_mat_trans = get_rotation_matrix(-an_x, 0.0, 0.0)

    #Create the normals
    normals = []
    for base_vec in base_vectors:
        base_vec = normalize_vector(base_vec)
        base_vec = matrix_vector_mult(rot_mat, base_vec)
        sp = xyz_to_sph(base_vec[0], base_vec[1], base_vec[2])
        for pan_curr in range(pan_min, pan_max + 1, d_x):
            for tilt_curr in range(tilt_min, tilt_max + 1, d_y):
                d_az = pan_curr*math.pi/180
                d_elev = tilt_curr*math.pi/180
                new_sp = add_angles_to_sph(sp, d_elev, d_az)
                new_normal = sph_to_xyz(new_sp[0], new_sp[1], new_sp[2])
                result_normal = matrix_vector_mult(rot_mat_trans, new_normal)
                normals.append(round_vector_elements(result_normal))


    #Write the created normals to a file
    output_string = ""
    for normal in normals:
        for idx,nor in enumerate(normal):
            if idx < 2:
                output_string += str(nor) + ","
            else:
                output_string += str(nor)
        output_string += ";"

    print 'Created normals:'
    print output_string
    print '--------------------'

    fh = open(rospack.get_path('asr_object_database') + output_rel_path, 'w')
    fh.writelines(output_string)
    fh.close()
    
    #Create a marker array from the created normals
    markerArray = MarkerArray()
    marker_id = 0
    size_vec  = len(base_vectors)
    color_step = 1
    if size_vec > 0:
        color_step = 1.0 / size_vec 
    
    for normal in normals:
        hue = (marker_id / (len(normals) / size_vec)) * color_step + 0.0
        rgb = colorsys.hsv_to_rgb(hue, 1.0, 1.0)

        marker = Marker()
        marker.id = marker_id
        marker.header.frame_id = marker_frame
        marker.ns = 'normals'
        marker.scale.x = 0.01
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = rgb[0]
        marker.color.g = rgb[1] 
        marker.color.b = rgb[2] 
        marker.color.a = 1.0
        
        point_start = Point()
        point_end = Point()
        point_end.x = normal[0]
        point_end.y = normal[1]
        point_end.z = normal[2]
        marker.points.append(point_start)
        marker.points.append(point_end)

        marker.type = marker.ARROW
        markerArray.markers.append(marker)
       
        marker_id = marker_id +1

    #Publish the normal markers
    print 'Publishing normals (Rate: ' + str(publish_rate) + ' Hz)'
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        publisher.publish(markerArray)
        rate.sleep()
