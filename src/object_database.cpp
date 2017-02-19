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
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include "object_database/ObjectDatabase.h"
#include "object_database/ObjectManager.h"

namespace fs = boost::filesystem;

int main(int argc, char** argv) {
    ros::init(argc, argv, "asr_object_database");

	// proper ros start
    ros::start();

    fs::path basePath = ros::package::getPath("asr_object_database");
	object_database::ObjectDatabase odb(basePath / "rsc/config.xml");

	ros::spin();

	// proper ros shutodwn
	ros::shutdown();

	return 0;
}


