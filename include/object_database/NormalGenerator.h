/**

Copyright (C) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heizmann Heinrich, Heller Florian, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Walter Milena

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

*/

#ifndef NORMAL_GENERATOR_H_
#define NORMAL_GENERATOR_H_

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>

namespace object_database {

    using namespace Eigen;

    class NormalGenerator {

    private:
        static Matrix3d getRotationMatrix(double angle_x, double angle_y, double angle_z) {
            Matrix3d m;
            m = AngleAxisd(angle_x, Vector3d::UnitX())
              * AngleAxisd(angle_y,  Vector3d::UnitY())
              * AngleAxisd(angle_z, Vector3d::UnitZ());

            return m;
        }

        static geometry_msgs::Point convertVectorFromEigen(Vector3d vec) {
            geometry_msgs::Point point;
            point.x = vec[0];
            point.y = vec[1];
            point.z = vec[2];
            return point;
        }

    public:
        static std::vector<geometry_msgs::Point> getNormalVectors(double angle_x, double angle_y, double angle_z, const std::string &savePath = std::string()) {
            //so far angle_z is not used

            std::vector<Vector3d> normals;
            Vector3d base_vec(0, 0, -1);

            normals.push_back(getRotationMatrix(0, 0, 0) * base_vec);

            normals.push_back(getRotationMatrix(angle_x, 0, 0) * base_vec);
            normals.push_back(getRotationMatrix(-angle_x, 0, 0) * base_vec);
            normals.push_back(getRotationMatrix(0, angle_y, 0) * base_vec);
            normals.push_back(getRotationMatrix(0, -angle_y, 0) * base_vec);

            normals.push_back(getRotationMatrix(angle_x, angle_y, 0) * base_vec);
            normals.push_back(getRotationMatrix(-angle_x, angle_y, 0) * base_vec);
            normals.push_back(getRotationMatrix(angle_x, -angle_y, 0) * base_vec);
            normals.push_back(getRotationMatrix(-angle_x, -angle_y, 0) * base_vec);

            std::vector<geometry_msgs::Point> geom_points;
            for (std::vector<Vector3d>::iterator iter = normals.begin(); iter != normals.end(); ++iter) {
                geom_points.push_back(convertVectorFromEigen(*iter));
            }

             //save calculated normals
            if (!savePath.empty()) {
                std::stringstream ss;
                for (std::vector<geometry_msgs::Point>::iterator iter = geom_points.begin(); iter != geom_points.end(); ++iter) {
                    geometry_msgs::Point normal = *iter;
                    ss << boost::lexical_cast<std::string>(normal.x) << "," << boost::lexical_cast<std::string>(normal.y) << "," << boost::lexical_cast<std::string>(normal.z) << ";";

                }
                std::string contents = ss.str();
                std::ofstream f(savePath.c_str());
                if ( !f ) {
                    ROS_DEBUG("Can't open file to save normals");
                }
                if ( !contents.empty() ) f << contents;
            }

            return geom_points;
        }
    };

}

#endif /* NORMAL_GENERATOR_H_ */
