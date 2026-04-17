#include <Sync.hpp>

Sync::Sync(std::string syncPath, std::vector<std::string> syncOrder)
    : syncPath_(syncPath), syncOrder_(syncOrder)
{
    // Check if the path exists
    if (!boost::filesystem::exists(syncPath_))
    {
        std::cerr << "Path " << syncPath_ << " does not exist" << std::endl;
        exit(1);
    }
}

std::string Sync::getCorrespondence(const std::string &targetSensor, const std::string &sourceSensor, const std::string &timestampRef)
{
    std::string timestamp = "-1";

    // Get the source and target columns
    int sourceColumn = std::distance(syncOrder_.begin(), std::find(syncOrder_.begin(), syncOrder_.end(), sourceSensor));
    int targetColumn = std::distance(syncOrder_.begin(), std::find(syncOrder_.begin(), syncOrder_.end(), targetSensor));
    
    // Validate the columns before proceeding
    if (sourceColumn >= syncOrder_.size() || targetColumn >= syncOrder_.size())
    {
        std::cerr << "Error: Invalid source or target sensor.\n";
        return timestamp;
    }

    // Validate the columns before proceeding
    if (sourceColumn >= syncOrder_.size() || targetColumn >= syncOrder_.size())
    {
        std::cerr << "Error: Invalid source or target sensor.\n";
        return timestamp;
    }

    try
    {
        // Open the sync file
        std::ifstream file(syncPath_);
        if (!file.is_open())
        {
            std::cerr << "Error: Could not open sync file.\n";
            return "";
        }

        std::string line;
        while (std::getline(file, line))
        {
            std::vector<std::string> parts;
            std::string part;
            std::istringstream stream(line);

            // Split line into parts
            while (stream >> part)
            {
                parts.push_back(part);
            }

            // Check if the reference timestamp is in the specified column
            if (sourceColumn < parts.size() && parts[sourceColumn].find(timestampRef) != std::string::npos)
            {
                if (targetColumn < parts.size())
                {
                    timestamp = parts[targetColumn];
                }
                break;
            }
        }
        file.close();

        // Return the found timestamp and reference timestamp
        return timestamp;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Warning: Corresponding timestamp not found for " << sourceSensor << "\n";
        return timestamp;;
    }
}

// Trim function to remove leading and trailing whitespace
std::string Sync::trim(const std::string &str)
{
    auto start = str.find_first_not_of(" \t\n\r\f\v");
    auto end = str.find_last_not_of(" \t\n\r\f\v");
    return (start == std::string::npos || end == std::string::npos) ? "" : str.substr(start, end - start + 1);
}

std::vector<std::string> Sync::toSyncOrder(const std::string &sync_order_str)
{
    std::vector<std::string> sync_order;

    std::stringstream ss(sync_order_str);
    std::string token;

    while (std::getline(ss, token, ' '))
    {
        // Trim whitespace from the token
        token = Sync::trim(token);
        // Remove all spaces within the token
        token.erase(std::remove(token.begin(), token.end(), ' '), token.end());

        if (!token.empty())
        {
            sync_order.push_back(token);
        }
    }

    return sync_order;
}