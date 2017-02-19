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

#ifndef TYPEDEF_HPP_
#define TYPEDEF_HPP_

#include <vector>
#include <map>
#include <boost/weak_ptr.hpp>

namespace object_database {
	class ObjectDatabaseConfig;
	typedef boost::shared_ptr<ObjectDatabaseConfig> ObjectDatabaseConfigPtr;

	class ObjectDatabaseRecognizer;
	typedef boost::shared_ptr<ObjectDatabaseRecognizer> ObjectDatabaseRecognizerPtr;
	typedef std::map<std::string, ObjectDatabaseRecognizerPtr> ObjectDatabaseRecognizerPtrMap;
	typedef std::vector<ObjectDatabaseRecognizerPtr> ObjectDatabaseRecognizerPtrCollection;

	class ObjectDatabaseEntry;
	typedef boost::shared_ptr<ObjectDatabaseEntry> ObjectDatabaseEntryPtr;
	typedef std::pair<std::string, ObjectDatabaseEntryPtr> ObjectDatabaseEntryPtrMapPair;
	typedef std::map<std::string, ObjectDatabaseEntryPtr> ObjectDatabaseEntryPtrMap;
	typedef std::vector<ObjectDatabaseEntryPtr> ObjectDatabaseEntryPtrCollection;
}

#endif /* TYPEDEF_HPP_ */
