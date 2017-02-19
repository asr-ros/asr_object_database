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

#ifndef OBJECTMANAGER_HPP_
#define OBJECTMANAGER_HPP_

#include <ros/ros.h>
#include <string>
#include <map>
#include "asr_object_database/ObjectMetaData.h"

namespace object_database {

    using namespace asr_object_database;

    typedef ObjectMetaData::Response ObjectTypeResponse;
	typedef ObjectTypeResponse::Ptr ObjectTypeResponsePtr;

	/**
	 * The ObjectManager class delivers a possibility to handle request to the object_database and cache their responses in a map.
	 */
	class ObjectManager {
	private:
		struct State {
		private:
			ros::NodeHandle mNodeHandle;
		public:
			ros::ServiceClient object_type_service_client;
            std::map<std::pair<std::string, std::string>, ObjectMetaData::Response::Ptr> object_type_cache;

			State() : mNodeHandle() {
                object_type_service_client = mNodeHandle.serviceClient<ObjectMetaData>("/asr_object_database/object_meta_data");
			}
		};

		static boost::shared_ptr<State> InstancePtr() {
			static const boost::shared_ptr<State> statePtr = boost::shared_ptr<State>(new State());
			return statePtr;
		}


	public:
		ObjectManager() {}

        ObjectTypeResponsePtr get(std::string objectTypeName, std::string recognizer) {
			boost::shared_ptr<State> statePtr = InstancePtr();

			if (!statePtr->object_type_service_client.exists()) {
				return ObjectTypeResponsePtr();
			}

            ObjectTypeResponsePtr responsePtr = statePtr->object_type_cache[std::make_pair(objectTypeName, recognizer)];

			if (!responsePtr) {          
                ObjectMetaData serviceCall;
				serviceCall.request.object_type = objectTypeName;
                serviceCall.request.recognizer = recognizer;
				statePtr->object_type_service_client.call(serviceCall);
				if (!serviceCall.response.is_valid) {
					return ObjectTypeResponsePtr();
				}

                responsePtr = ObjectTypeResponsePtr(new ObjectMetaData::Response(serviceCall.response));
                statePtr->object_type_cache[std::make_pair(objectTypeName, recognizer)] = responsePtr;
			}
			return responsePtr;
		}
	};
}



#endif /* OBJECTMANAGER_HPP_ */
