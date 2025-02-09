#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>

namespace Referee 
{
    namespace Utils 
    {
        namespace CoordinateSystem
        {
            enum class CoordinateSystem
            {
                UTM,    //Universal Transverse Mercator 
                DMS,    //Degrees, Minutes, Seconds
                DEG,    //Decimal Degrees
                LV95,   //Swiss Grid (translation of UTM)
            };
        } // CoordinateSystem

        /*
        @brief Convert latitude, longitude to Cartesian coordinates (X, Y)
        @param lat Latitude in degrees
        @param lon Longitude in degrees
        @param x X coordinate in meters
        @param y Y coordinate in meters
        @param FromCoordSys Coordinate system of input latitude and longitude
        @param ToCoordSys Coordinate system of output coordinates
        */
        namespace Conversions
        {
                void ConvertLatLonAltToCartesian(double lat, double lon, double alt, double &x, double &y, double &z, CoordinateSystem::CoordinateSystem fromCoordSys, CoordinateSystem::CoordinateSystem toCoordSys);
                void ConvertECEFToLatLonAlt(double x, double y, double z, double &lat, double &lon, double &alt);
        } // Conversions

        namespace FileIterators
        {
            /*
            @brief Get all files in a directory with a specific extension
            @param directory Directory to search for files
            @param extension File extension to search for
            */
            std::vector<std::string> GetFilesInDirectory(const std::string& directory, const std::string& extension);
        } // FileIterators
    } // Utils
}