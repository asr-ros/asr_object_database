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

#define BOOST_NO_CXX11_SCOPED_ENUMS

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <ros_uri/ros_uri.hpp>
#include "object_database/ObjectDatabase.h"
#include "object_database/ObjectDatabaseRecognizer.h"
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

namespace object_database
{
ObjectDatabase::ObjectDatabase(fs::path configurationFilePath) :
    mGlobalNodeHandle("asr_object_database"),
    mConfigurationFilePath(configurationFilePath)
{
    // Create the config and read it.
    mConfig.read(configurationFilePath);

    // Read all objects in database.
    this->readAllObjectDatabaseRecognizers();

    mRecognizerListServiceHandle =
            mGlobalNodeHandle.advertiseService("recognizer_list",
                                               &ObjectDatabase::processRecognizerListRequest, this);
    mObjectTypeListServiceHandle =
            mGlobalNodeHandle.advertiseService("object_type_list",
                                               &ObjectDatabase::processObjectTypeListRequest, this);
    mObjectMetaDataServiceHandle =
            mGlobalNodeHandle.advertiseService("object_meta_data",
                                               &ObjectDatabase::processObjectMetaDataRequest, this);


    mRecognizerListMeshesServiceHandle =
            mGlobalNodeHandle.advertiseService("recognizer_list_meshes",
                                               &ObjectDatabase::processRecognizerListMeshesRequest, this);

    mObjectTypeGeneratorServiceHandle =
            mGlobalNodeHandle.advertiseService("generate_object_type",
                                               &ObjectDatabase::processObjectTypeGeneratorRequest, this);

}

bool ObjectDatabase::processRecognizerListRequest(RecognizerList::Request &req,
                                                      RecognizerList::Response &res)
{
    // get all object types and add it to the response collection
    ObjectDatabaseRecognizerPtrMap objectTypes = mConfig.getObjectCategories();
    BOOST_FOREACH(ObjectDatabaseRecognizerPtrMap::value_type mapTuple, objectTypes)
    {
        res.recognizer_list.push_back(mapTuple.second->getKeyword());
    }

    // in the end, sort by name
    std::sort(res.recognizer_list.begin(), res.recognizer_list.end());

    return true;
}

bool ObjectDatabase::processObjectTypeListRequest(ObjectTypeList::Request &req,
                                                  ObjectTypeList::Response &res)
{
    // get the parameter passed by the request
    std::string recognizerName = req.recognizer;

    // creating a collection
    ObjectDatabaseRecognizerPtrCollection collection;

    if (recognizerName == "all")
    {
        // if the user wants all items, replace the collection
        // by another collection containing all object types
        BOOST_FOREACH(ObjectDatabaseRecognizerPtrMap::value_type mapVal, mConfig.getObjectCategories())
        {
            collection.push_back(mapVal.second);
        }
    } else
    {
        // in any other case try to find the object type object
        ObjectDatabaseRecognizerPtr recognizerPtr = mConfig.getRecognizer(recognizerName);

        // creating null pointer reference
        ObjectDatabaseRecognizerPtr nullPtr;
        if (recognizerPtr == nullPtr)
        {
            ROS_DEBUG("No object type definition for name '%s' found", recognizerName.c_str());
            return false;
        }

        // add object type object to collection, if there is a valid object
        collection.push_back(recognizerPtr);
    }

    ROS_DEBUG("Collection contains %lu ObjectTypes", collection.size());

    // Loop through the collection and read each object type and add its entries to the response
    BOOST_FOREACH(ObjectDatabaseRecognizerPtr recognizerPtr, collection)
    {
        // reading objects into object type
        recognizerPtr->readEntries();

        // adding results to response
        ObjectDatabaseEntryPtrMap entryPtrMap = recognizerPtr->getEntries();
        BOOST_FOREACH(ObjectDatabaseEntryPtrMapPair entryPtrMapPair, entryPtrMap)
        {
            res.object_type_list.push_back(entryPtrMapPair.first);
        }
    }

    // sorting the object for the response
    std::sort(res.object_type_list.begin(), res.object_type_list.end());

    ROS_DEBUG("Object List contains %lu items", res.object_type_list.size());
    ROS_DEBUG("Reached end of processing object list request");

    return true;
}

bool ObjectDatabase::processObjectMetaDataRequest(ObjectMetaData::Request &req, ObjectMetaData::Response &res)
{
    ObjectDatabaseRecognizerPtrMap recognizerPtrMap = mConfig.getObjectCategories();
    std::string object_type = req.object_type;
    std::string recognizer = req.recognizer;

    // object information
    res.object_folder = std::string();
    res.is_valid = false;

    for(ObjectDatabaseRecognizerPtrMap::value_type val : recognizerPtrMap) {
        ObjectDatabaseEntryPtr objectEntryPtr = val.second->getEntry(object_type);
        if ((objectEntryPtr) && (objectEntryPtr->getRecognizer()->getKeyword() == recognizer)){
            res.object_folder = ros_uri::package_uri(objectEntryPtr->getPath().string(), "asr_object_database");
            res.object_mesh_resource = ros_uri::package_uri(objectEntryPtr->getRvizMeshResourcePath().string(), "asr_object_database");
            res.normal_vectors = objectEntryPtr->getNormalVectors();
            res.deviations = objectEntryPtr->getDeviationsFromFile();
            res.is_valid = true;
            res.is_rotation_invariant = objectEntryPtr->getRotationInvarianceFromFile();
            break;
        }
    }

    return true;
}

bool ObjectDatabase::processRecognizerListMeshesRequest(RecognizerListMeshes::Request &req, RecognizerListMeshes::Response &res) {
    // get the parameter passed by the request
    std::string recognizerName = req.recognizer;

    // creating a collection
    ObjectDatabaseRecognizerPtrCollection collection;

    if (recognizerName == "all")
    {
        // if the user wants all items, replace the collection
        // by another collection containing all object types
        BOOST_FOREACH(ObjectDatabaseRecognizerPtrMap::value_type mapVal, mConfig.getObjectCategories())
        {
            collection.push_back(mapVal.second);
        }
    } else
    {
        // in any other case try to find the object type object
        ObjectDatabaseRecognizerPtr recognizerPtr = mConfig.getRecognizer(recognizerName);

        // creating null pointer reference
        ObjectDatabaseRecognizerPtr nullPtr;
        if (recognizerPtr == nullPtr)
        {
            ROS_DEBUG("No object type definition for name '%s' found", recognizerName.c_str());
            return false;
        }

        // add object type object to collection, if there is a valid object
        collection.push_back(recognizerPtr);
    }

    ROS_DEBUG("Collection contains %lu ObjectTypes", collection.size());

    // Loop through the collection and read each object type and add its entries to the response
    BOOST_FOREACH(ObjectDatabaseRecognizerPtr recognizerPtr, collection)
    {
        // reading objects into object type
        recognizerPtr->readEntries();

        // adding results to response
        ObjectDatabaseEntryPtrMap entryPtrMap = recognizerPtr->getEntries();
        BOOST_FOREACH(ObjectDatabaseEntryPtrMapPair entryPtrMapPair, entryPtrMap)
        {
            std::string mesh_path = ros_uri::package_uri(entryPtrMapPair.second->getRvizMeshResourcePath().string(), "asr_object_database");
            if (mesh_path != "package://asr_object_database/") {
                res.meshes.push_back(mesh_path);
            }
        }
    }


    ROS_DEBUG("Object meshes list contains %lu items", res.meshes.size());
    ROS_DEBUG("Reached end of processing object meshes request");

    return true;
}

bool ObjectDatabase::processObjectTypeGeneratorRequest(ObjectTypeGenerator::Request &req, ObjectTypeGenerator::Response &res)
{
    //check if given file exists
    const fs::path sourcepath(req.sourcefile.c_str());
    if (!fs::exists(sourcepath))
    {
        ROS_ERROR("File does not exist.");
        return false;
    }

    //check if given file ends with _tex.obj
    std::string fileending = req.sourcefile.substr(req.sourcefile.find_last_of("_"));
    if (fileending.compare("_tex.obj") != 0)
    {
        ROS_ERROR("File has to end with '_tex.obj.'");
        return false;
    }

    //check for the .mtl file in given directory
    std::string::size_type const q(req.sourcefile.find_last_of('.'));
    const fs::path mtlSourcePath ((req.sourcefile.substr(0, q) + ".obj.mtl").c_str());
    if (!fs::exists(mtlSourcePath))
    {
        ROS_ERROR("No .mtl file can be found in given directory.");
        return false;
    }

    //extract filename from given path
    std::string extfilename = req.sourcefile.substr(req.sourcefile.find_last_of("/\\") + 1);
    std::string::size_type const p(extfilename.find_first_of('_'));
    std::string filename = extfilename.substr(0, p);

    //get path to this package
    std::string packagepath = ros::package::getPath("asr_object_database");

    //copy source file to database
    std::string predestpath = packagepath + "/rsc/databases/textured_objects/" + filename + "/" + filename + ".obj";
    const fs::path destpath(predestpath.c_str());

    std::string prenewDirectory = packagepath + "/rsc/databases/textured_objects/" + filename;
    const fs::path newDirectory(prenewDirectory.c_str());

    if (!fs::exists(newDirectory))
    {
        fs::create_directory(newDirectory);
    }

    fs::copy_file(sourcepath, destpath, fs::copy_option::overwrite_if_exists);

    //generate destination path for the .mtl file
    std::string::size_type const r(extfilename.find_last_of('.'));
    std::string premtlDestPath = prenewDirectory + "/" + extfilename.substr(0, r) + ".obj.mtl";
    const fs::path mtlDestPath = premtlDestPath.c_str();

    //copy .mtl file
    fs::copy_file(mtlSourcePath, mtlDestPath, fs::copy_option::overwrite_if_exists);

    //generate source path of the .png file
    std::string prepngSourcePath = req.sourcefile.substr(0, q) + ".png";
    const fs::path pngSourcePath = prepngSourcePath.c_str();

    if (!fs::exists(pngSourcePath))
    {
        ROS_ERROR("No .png file could be found in the given directory.");
        return false;
    }

    //generate destination path for the .png file
    std::string prepngDestPath = prenewDirectory + "/" + extfilename.substr(0, r) + ".png";
    const fs::path pngDestPath = prepngDestPath.c_str();

    //copy .png file
    if (!fs::exists(pngDestPath))
    {
        fs::copy_file(pngSourcePath, pngDestPath);
    }

    //create .dae file
    std::string predaefilepath = packagepath + "/rsc/databases/textured_objects/" + filename + "/" + filename + ".dae";
    const fs::path daefilepath(predaefilepath.c_str());
    if (!fs::exists(daefilepath))
    {
        std::string databaseFolder = prenewDirectory + "/";
        const char *navigateToDatabaseFolder = databaseFolder.c_str();
        chdir(navigateToDatabaseFolder);
        std::string mlsoutput = filename + ".dae";
        const char *meshlabserver = ("meshlabserver -i " + filename+".obj" + " -o " + mlsoutput +
                                     " -om vc vn wt").c_str();
        system(meshlabserver);
    }
    return true;
}

void ObjectDatabase::readAllObjectDatabaseRecognizers()
{
    ObjectDatabaseRecognizerPtrMap recognizerMap = mConfig.getObjectCategories();
    ROS_DEBUG("Start reading all object types");
    BOOST_FOREACH(ObjectDatabaseRecognizerPtrMap::value_type mapValue, recognizerMap)
    {
        ROS_DEBUG("Reading %s", mapValue.first.c_str());
        mapValue.second->readEntries();
    }
}
}
