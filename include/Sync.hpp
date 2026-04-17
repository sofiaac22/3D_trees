#ifndef SYNC_HPP
#define SYNC_HPP

#include <iostream>
#include <sstream>
#include <boost/filesystem.hpp>

class Sync
{
public:
    /**
     * @brief Constructor of Sync class, it receives a path to a file and a
     * vector of strings with the order of the sync
     *
     * @param syncPath Global path to the sync.txt file
     * @param syncOrder Vector of strings with the order of the sync (parameters.json)
     */
    Sync(std::string syncPath, std::vector<std::string> syncOrder);

    /** 
     * @brief This method returns the correspondence timestamp between two sensors
     * 
     * @param targetSensor Name of the target sensor. This means the sensor that will be synchronized
     * @param sourceSensor Name of the source sensor. This means the sensor that will be used as reference
     * @param timestamp Timestamp of the source sensor
     */
    std::string getCorrespondence(const std::string &targetSensor, const std::string &sourceSensor, const std::string &timestampRef);

    /**
     * @brief This method is used to split a string into a vector of strings
     * 
     * @param sync_order_str String to be split
     */
    static std::vector<std::string> toSyncOrder(const std::string &sync_order_str);

    /**
     * @brief This method trims the whitespace from a string
     * 
     * @param str String to be trimmed
     */
    static std::string trim(const std::string& str);

private:
    std::string syncPath_;
    std::vector<std::string> syncOrder_;

};

#endif // SYNC_HPP