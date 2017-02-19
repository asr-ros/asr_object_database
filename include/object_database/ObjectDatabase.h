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

#ifndef OBJECTDATABASE_H_
#define OBJECTDATABASE_H_

#include <string>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include "typedef.h"
#include "ObjectDatabaseConfig.h"
#include "asr_object_database/RecognizerList.h"
#include "asr_object_database/ObjectTypeList.h"
#include "asr_object_database/ObjectMetaData.h"
#include "asr_object_database/RecognizerListMeshes.h"
#include "asr_object_database/ObjectTypeGenerator.h"


namespace object_database
{
using namespace asr_object_database;

namespace fs = boost::filesystem;
class ObjectDatabase
{
    /**
     * Global node handle.
     */
    ros::NodeHandle mGlobalNodeHandle;

    /**
     * the recognizer list service handle
     */
    ros::ServiceServer mRecognizerListServiceHandle;

    /**
     * the recognizer meshes list service handle
     */
    ros::ServiceServer mRecognizerListMeshesServiceHandle;

    /**
     * the object type list service handle
     */
    ros::ServiceServer mObjectTypeListServiceHandle;

    /**
     * the object type service handle
     */
    ros::ServiceServer mObjectMetaDataServiceHandle;

    /**
     * the object type generator service handle
     */
    ros::ServiceServer mObjectTypeGeneratorServiceHandle;

    /**
     * The path to the configuration file.
     */
    fs::path mConfigurationFilePath;

    /**
     * the config file.
     */
    ObjectDatabaseConfig mConfig;
public:
    /**
     * ctor
     * @param configurationFilePath path to config file.
     */
    ObjectDatabase(fs::path configurationFilePath);

    /**
     * dtor
     */
    virtual ~ObjectDatabase()
    {}

    /**
     * Processes the request to get a list of recognizers.
     * @param req the request message
     * @param res the correlated response message.
     */
    bool processRecognizerListRequest(RecognizerList::Request &req, RecognizerList::Response &res);

    /**
     * Processes the request to get a list of entries in an arbitrary recognizer.
     * @param req the request message
     * @param res the correlated response message.
     */
    bool processObjectTypeListRequest(ObjectTypeList::Request &req, ObjectTypeList::Response &res);

    /**
     * Process the request to get the path of the object
     * @param req the request message
     * @param res the correlated response message.
     */
    bool processObjectMetaDataRequest(ObjectMetaData::Request &req, ObjectMetaData::Response &res);

    /**
     * Process the request to get all meshes of a recognizer
     * @param req the request message
     * @param res the correlated response message.
     */
    bool processRecognizerListMeshesRequest(RecognizerListMeshes::Request &req, RecognizerListMeshes::Response &res);

    /**
     * Processes the request to create a new object folder in textured objects
     * @param req the request message contains the source path of the object file
     * @param res the response message will be empty
     * @return
     */
    bool processObjectTypeGeneratorRequest(ObjectTypeGenerator::Request &req, ObjectTypeGenerator::Response &res);


    /**
     * Convenience function for calling readObjectType with all object types.
     * @see RecognitionManager::readObjectType
     */
    void readAllObjectDatabaseRecognizers();
};
}


#endif /* OBJECTDATABASE_H_ */
