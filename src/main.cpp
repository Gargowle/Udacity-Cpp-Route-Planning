#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}


float GetValidFloat(std::string displayname)
{ 
  float input_min = 0.0f;
  float input_max = 100.0f;

  float inputvalue;
  std::string input_line;
  while(true)
  {
    std::cout << "Please enter a numeric value for " << displayname << " in the range " << input_min << " to " << input_max << "." << std::endl;
    
    // input handling adapted from https://stackoverflow.com/questions/10828937/how-to-make-cin-take-only-numbers
    // idea: always read out whole line and make sure that this entire line can be converted to a float
    // this catches faulty inputs robustly;
    if(std::getline(std::cin, input_line))
    {
      std::stringstream input_string_stream(input_line);
      if(input_string_stream >> inputvalue)
      {
        if(input_string_stream.eof() // the whole line has been consumed to get inputvalue
           && (inputvalue >= input_min && inputvalue <= input_max) ) // input in legal range
        {
          // inputvalue is legal and is ready to be returned
          break;
        }
      }
      std::cout << "Entered value is illegal!" << std::endl;
    }
  }
  return inputvalue;
}


int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    float start_x = GetValidFloat("Start X");
    float start_y = GetValidFloat("Start Y");
    float end_x = GetValidFloat("End X");
    float end_y = GetValidFloat("End Y");
    
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
