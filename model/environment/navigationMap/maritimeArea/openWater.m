function Map=openWater()
%OPENWATER   Generate empty (open water) environment map structure for simulation.
%
%   Map = openWater() returns a structure with default (empty) fields for
%   environmental map and boundary definitions, representing an unbounded 
%   open water area with no explicit navigation or obstacle regions.
%
%   Outputs:
%     Map      : Structure with fields
%                 - range         : (empty) simulation range/bounding box
%                 - boundary      : (empty) map boundary polygon
%                 - navArea       : (empty) navigable area (polygon)
%                 - unnaviNavArea : (empty) unnavigable area (polygon)
%                 - wind          : (empty) wind field data (user-defined)
%                 - wave          : (empty) wave field data (user-defined)
%                 - current       : (empty) current field data (user-defined)
%
%   Description:
%     - Serves as a template for open sea or unbounded simulation environments.
%     - Fields can be populated or replaced with specific map, weather, or 
%       boundary data for scenario-based simulations.
%
%   Author: Wenxiang Wu
%   Date:   2025-02-25
%
%   Example usage:
%     Map = openWater();
Map.range=[];
Map.boundary=[];
Map.navArea=[];
Map.unnaviNavArea=[];
Map.wind=[];
Map.wave=[];
Map.current=[];
end
