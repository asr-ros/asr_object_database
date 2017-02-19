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

#ifndef OBJECT_DATABASE_CONFIG_H_
#define OBJECT_DATABASE_CONFIG_H_

#include <boost/filesystem.hpp>
#include <string>
#include "typedef.h"

namespace object_database {
	namespace fs = boost::filesystem;
	class ObjectDatabaseConfig {
		typedef boost::shared_ptr<ObjectDatabaseConfig> ObjectDatabaseConfigPtr;
	public:
		/**
		 * Creates a new ObjectDatabaseConfig.
		 * @param configFile the path to the config on the disk.
		 */
		ObjectDatabaseConfig(const fs::path configFile = fs::path());

		// dtor
		virtual ~ObjectDatabaseConfig() { }

		/**
		 * Reads the configuration XML file.
		 * @param config_file the path to load the config file from
		 * @see ../object_database/object_database_config.xml
		 */
		bool read(const fs::path config_file);

		/**
		 * @return the object recognizer collection
		 */
		const ObjectDatabaseRecognizerPtrMap getObjectCategories();

		/**
		 * Searching for the right object recognizer for given recognizer name.
		 * @param recognizerName the name of the searched type
		 * @return the object type or the nullPtr
		 */
		const ObjectDatabaseRecognizerPtr getRecognizer(const std::string recognizerName);
	private:
		/**
		 * Pointer to the object types collection.
		 */
		ObjectDatabaseRecognizerPtrMap mObjectCategories;
	};
}

#endif /* OBJECT_DATABASE_CONFIG_H_ */
