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

#ifndef OBJECT_DATABASE_ENTRY_H_
#define OBJECT_DATABASE_ENTRY_H_

#include <boost/filesystem.hpp>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "ObjectDatabaseRecognizer.h"

namespace object_database
{
namespace fs = boost::filesystem;

/**
* The class 'ObjectDatabaseEntry' contains all information about an entry in the
* object database.
*/
class ObjectDatabaseEntry
{
public:
    /**
    * Creates a new database entry.
    * @param recognizerPtr raw pointer to the type this entry belongs to -
    *  the passer guarantees for the lifetime of this pointer.
    * @param uniqueName the unique name this entry is identified by.
    * @param path the path to the entries definition file or directory.
    */
    ROS_DEPRECATED ObjectDatabaseEntry(ObjectDatabaseRecognizer* recognizerPtr, const std::string uniqueName,
                        const fs::path path, const fs::path rvizMeshResourcePath,
                        const fs::path normalVectorResourcePath);
    ObjectDatabaseEntry(ObjectDatabaseRecognizer *recognizerPtr, const std::string uniqueName,
                        const fs::path path, const fs::path rvizMeshResourcePath,
                        const fs::path normalVectorResourcePath, const fs::path getDeviationResourcePath,
                        const fs::path rotation_invariance_resource_path);

    // dtor
    virtual ~ObjectDatabaseEntry()
    { }

    /**
    * @return the raw pointer to the object recognizer.
    */
    ObjectDatabaseRecognizer* getRecognizer();

    /**
    * @return the unique name of the entry
    */
    const std::string getUniqueName();

    /**
    * @return the path to the entries definition file or directory.
    */
    const fs::path getPath();

    /**
    * @return the path to the entries definition file or directory.
    */
    const fs::path getRvizMeshResourcePath();

    /**
    * @return the path to the normal vector definition file.
    */
    const fs::path getNormalVectorResourcePath();

    std::vector<geometry_msgs::Point> getNormalVectors();

    fs::path getDeviationResourcePath() const;

    std::vector<double> getDeviationsFromFile();
    

    fs::path getRotationInvarianceResourcePath() const;

    /**
    * @return whether the object is rotation invariant.
    */
    const bool getRotationInvarianceFromFile();
    
private:
    /**
    * Raw pointer to Object Database recognizer.
    */
    ObjectDatabaseRecognizer* mRecognizerPtr;

    /**
    * String containing the unique name.
    */
    std::string mUniqueName;

    /**
    * Path containing the definition path or directory.
    */
    fs::path mPath;

    /**
    * Path containing the definition path or directory.
    */
    fs::path mRvizMeshResourcePath;

    /**
    * Path containing the definition of normal vector.
    */
    fs::path mNormalVectorResourcePath;

    /**
    * Path containing the definition of deviation
    */
    fs::path deviation_resource_path_;

    /**
    * Path containing the definition of rotation invariance
    */
    fs::path rotation_invariance_resource_path;

    std::vector<geometry_msgs::Point> mNormalVectors;
    std::vector<double> deviations_;
    bool rotation_invariant;
};
}

#endif
