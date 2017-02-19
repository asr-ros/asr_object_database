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
#include <boost/regex.hpp>
#include "object_database/ObjectDatabaseRecognizer.h"
#include <ros_uri/ros_uri.hpp>
#include <boost/algorithm/string.hpp>

namespace object_database
{
ObjectDatabaseRecognizer::ObjectDatabaseRecognizer(database db) :
    mDatabase(db),
    mRead(false)
{ }

const std::string ObjectDatabaseRecognizer::getKeyword()
{
    return mDatabase.uniqueName();
}

void ObjectDatabaseRecognizer::setKeyword(const std::string kw)
{
    mDatabase.uniqueName(kw);
}

const fs::path ObjectDatabaseRecognizer::getPath()
{
    std::string realPath = ros_uri::absolute_path(mDatabase.objectDbRootFolder());
    return fs::path(realPath);
}

void ObjectDatabaseRecognizer::setPath(const fs::path pt)
{
    std::string packageString = ros_uri::package_uri(pt.string());
    mDatabase.objectDbRootFolder(packageString);
}

const std::string ObjectDatabaseRecognizer::getRegexp()
{
    return mDatabase.basenameMatchPattern();
}

void ObjectDatabaseRecognizer::setRegexp(const std::string regexp)
{
    mDatabase.basenameMatchPattern(regexp);
}

bool ObjectDatabaseRecognizer::isRead()
{
    return mRead;
}

void ObjectDatabaseRecognizer::setRead(bool rd)
{
    mRead = rd;
}

void ObjectDatabaseRecognizer::readEntries()
{
    // get the path where the database objects can be found
    fs::path folder = this->getPath();
    std::string regexp = this->getRegexp();

    // if the folder path is no directory or if the folder is already read
    if (!fs::is_directory(folder))
    {
        ROS_DEBUG("'%s' is not a valid folder", folder.c_str());
        return;
    } else if (this->isRead())
    {
        ROS_DEBUG("'%s' already read", folder.c_str());
        return;
    }

    // Matching the expression regexp
    boost::regex expression(regexp);


    for (fs::directory_iterator diriter(folder); diriter != fs::directory_iterator(); diriter++)
    {
        fs::path path = *diriter;

        fs::path filename = path.filename();
        boost::cmatch what;
        if (boost::regex_match(filename.c_str(), what, expression))
        {
            if (fs::is_directory(path))
            {
                std::string strUniqueName;

                // generate the filename
                if (what.size() > 1)
                {
                    strUniqueName = what[1].str();
                } else
                {
                    strUniqueName = what[0].str();
                }

                fs::path rvizMeshResourcePath = fs::path();
                fs::path normalVectorResourcePath = fs::path();
                fs::path deviation_resource_path = fs::path();
                fs::path rotation_invariance_resource_path = fs::path();
                for (fs::directory_iterator subdiriter(path); subdiriter != fs::directory_iterator();
                     subdiriter++)
                {
                    fs::path subdir_path = *subdiriter;
                    fs::path subdir_filename = subdir_path.filename();

                    boost::cmatch match;
                    if (boost::regex_match(subdir_filename.c_str(), match, boost::regex(".*\\.dae")))
                    {
                        rvizMeshResourcePath = subdir_path;
                    } else
                    {
                        if (boost::regex_match(subdir_filename.c_str(), match, boost::regex(".*\\.nv.txt")))
                        {
                            normalVectorResourcePath = subdir_path;
                        }
                        if (boost::regex_match(subdir_filename.c_str(), match, boost::regex(".*\\.dev.txt")))
                        {
                            deviation_resource_path = subdir_path;
                        }
                        if (boost::regex_match(subdir_filename.c_str(), match, boost::regex(".*\\.ri.txt")))
                        {
                            rotation_invariance_resource_path = subdir_path;
                        }
                    }
                }

                ObjectDatabaseEntryPtr entryPtr(new ObjectDatabaseEntry(this, strUniqueName,
                                                                        path, rvizMeshResourcePath,
                                                                        normalVectorResourcePath,
                                                                        deviation_resource_path,
                                                                        rotation_invariance_resource_path));
                mEntries[entryPtr->getUniqueName()] = entryPtr;

                ROS_DEBUG("File '%s' in db called '%s'", filename.c_str(), strUniqueName.c_str());
            }
        }
    }
    this->setRead(true);
}

ObjectDatabaseEntryPtrMap ObjectDatabaseRecognizer::getEntries()
{
    return mEntries;
}

const ObjectDatabaseEntryPtr& ObjectDatabaseRecognizer::getEntry(const std::string objectName)
{
    ObjectDatabaseEntryPtr ptr = this->mEntries[objectName];
    return *(new ObjectDatabaseEntryPtr(ptr));
}
}

