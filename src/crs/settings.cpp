/**
 * @file        settings.cpp
 * 
 * @brief       
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-03-12
 */
#include <crs/settings.hpp>

#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <filesystem>


crs::Settings crs::LoadSettings(const std::string& Filename)
{
    // Parse the JSON file
    std::ifstream Stream;
    Stream.open(Filename, std::ios::in);
    if (!Stream.is_open())
    {
        std::stringstream ss;
        ss << "Cannot open file " << Filename << " for reading.";
        throw std::runtime_error(ss.str());
    }
    nlohmann::json json = nlohmann::json::parse(Stream);
    Stream.close();

    // Initialize the default settings
    crs::Settings Settings;
    Settings.DistanceFunction = crs::DistanceBackend::DIJKSTRA;
    Settings.HamiltonianBias = crs::AdjacencyOrdering::DISTANCE;
    Settings.MultipleComponents = true;
    Settings.ForceHamiltonianPath = false;
    Settings.SIGSubGraph = true;

    // Load the settings
    // Input mesh and samples
    if (!json.contains("mesh") || !json.contains("samples"))
        throw std::runtime_error("Settings file must contain \"mesh\" and \"samples\" attribute.");
    if (!json["mesh"].is_string() || !json["samples"].is_string())
        throw std::runtime_error("Attributes \"mesh\" and \"samples\" must be strings.");
    Settings.InputMesh = json["mesh"];
    Settings.InputSamples = json["samples"];
    // Output prefix, if given
    if (json.contains("output_prefix"))
    {
        if (!json["output_prefix"].is_string())
            throw std::runtime_error("Attribute \"output_prefix\" must be a string.");
        Settings.OutputPrefix = json["output_prefix"];
    }
    else
    {
        // Use the input mesh basename
        Settings.OutputPrefix = std::filesystem::path(Settings.InputMesh).replace_extension("").string();
        Settings.OutputPrefix += "-";
    }

    // Flags
    if (json.contains("use_sig"))
    {
        if (!json["use_sig"].is_boolean())
            throw std::runtime_error("Attribute \"use_sig\" must be a boolean.");
        Settings.SIGSubGraph = json["use_sig"];
    }
    if (json.contains("multi_components"))
    {
        if (!json["multi_components"].is_boolean())
            throw std::runtime_error("Attribute \"multi_components\" must be a boolean.");
        Settings.MultipleComponents = json["multi_components"];
    }
    if (json.contains("force_hamiltonian"))
    {
        if (!json["force_hamiltonian"].is_boolean())
            throw std::runtime_error("Attribute \"force_hamiltonian\" must be a boolean.");
        Settings.ForceHamiltonianPath = json["force_hamiltonian"];
    }

    // Enums
    if (json.contains("distance"))
    {
        if (!json["distance"].is_string())
            throw std::runtime_error("Attribute \"distance\" must be a string.");
        std::string Distance = json["distance"];
        std::transform(Distance.begin(), Distance.end(), Distance.begin(), [](char c) { return (char)std::tolower(c); });
        if (Distance == "dijkstra")
            Settings.DistanceFunction = crs::DistanceBackend::DIJKSTRA;
        else if (Distance == "heat")
            Settings.DistanceFunction = crs::DistanceBackend::HEAT;
        else if (Distance == "exact")
            Settings.DistanceFunction = crs::DistanceBackend::EXACT;
        else
        {
            std::stringstream ss;
            ss << "Attribute \"distance\" only admits values \"dijkstra\", \"heat\", and \"exact\". ";
            ss << '\"' << json["distance"] << "\" is not an admissible value.";
            throw std::runtime_error(ss.str());
        }
    }
    if (json.contains("bias"))
    {
        if (!json["bias"].is_string())
            throw std::runtime_error("Attribute \"bias\" must be a string.");
        std::string Bias = json["bias"];
        std::transform(Bias.begin(), Bias.end(), Bias.begin(), [](char c) { return (char)std::tolower(c); });
        if (Bias == "none")
            Settings.HamiltonianBias = crs::AdjacencyOrdering::RANDOM;
        else if (Bias == "degree")
            Settings.HamiltonianBias = crs::AdjacencyOrdering::DEGREE;
        else if (Bias == "distance")
            Settings.HamiltonianBias = crs::AdjacencyOrdering::DISTANCE;
        else
        {
            std::stringstream ss;
            ss << "Attribute \"bias\" only admits values \"none\", \"degree\", and \"distance\". ";
            ss << '\"' << json["bias"] << "\" is not an admissible value.";
            throw std::runtime_error(ss.str());
        }
    }

    return Settings;
}