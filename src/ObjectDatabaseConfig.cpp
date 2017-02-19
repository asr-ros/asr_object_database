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

#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
#include <boost/foreach.hpp>
#include "object_database/ObjectDatabaseConfig.h"
#include "object_database/ObjectDatabaseRecognizer.h"

namespace object_database {
	ObjectDatabaseConfig::ObjectDatabaseConfig(const fs::path configFile) {
		if (configFile != fs::path()) {
			this->read(configFile);
		}
	}

	bool ObjectDatabaseConfig::read(const fs::path configFile) {
		try {
            std::unique_ptr<configuration> config = configuration_(configFile.string());

			for (configuration::database_iterator iter = config->database().begin(); iter != config->database().end(); iter++) {
				database db = *iter;

				ObjectDatabaseRecognizerPtr recognizerItemPtr(new ObjectDatabaseRecognizer(db));

				mObjectCategories[db.uniqueName()] = recognizerItemPtr;
			}
		} catch (xml_schema::exception& e) {
			std::cerr << e << std::endl;
			return false;
		}
		return true;
	}

	const ObjectDatabaseRecognizerPtrMap ObjectDatabaseConfig::getObjectCategories() {
		return mObjectCategories;
	}

	const ObjectDatabaseRecognizerPtr ObjectDatabaseConfig::getRecognizer(const std::string recognizerName) {
		ObjectDatabaseRecognizerPtr ptr = mObjectCategories[recognizerName];
		return ptr;
	}
}
