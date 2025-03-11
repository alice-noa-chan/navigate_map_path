## Introduce
A program that searches for the shortest route based on the origin and destination coordinates entered by the user and visualizes the result on the map. It supports various route search algorithms (A*, Dijkstra, Greedy Best-First Search, BFS, and DFS) and displays the search results on the map using Folium. It also provides the ability to check the results directly from the Jupiter Notebook.

## Functions
- **Download OSM data**: Download road data around origin and destination via Overpass API.
- **Graph configuration**: Pars the downloaded OSM data into a graph with nodes and edges.
- **Route navigation**: Use the algorithm below to explore the shortest path.
  - A* (Using Haversine Heuristic)
  - Dijkstra
  - Greedy Best-First Search
  - BFS (Breadth-First Search)
  - DFS (Depth-First Search)
- **Map visualization**: Use Folium to display the discovered path on the map and save it as an HTML file.
- **Jupyter Notebook Support**: Navigation results are available right on Jupyter Notebook.

## How to install and run?
### 1. Installing the required library
To run the program, use the following commands to install the required library:
```bash
pip install folium requests
```

### 2. Running a program
- Set the origin and destination coordinates (latitude, longitude) within the code.
- Run the program from the terminal or run the code from the Jupiter Notebook.

### 3. Select algorithms
When the program runs, the following algorithm selection menu appears: Enter one of the following:
- **1**: A* (Haversine Heuristic)
- **2**: Dijkstra
- **3**: Greedy Best-First Search
- **4**: BFS
- **5**: DFS
- **6**: Run all algorithms

### 4. Check the results
- The console outputs the total travel distance of the route.
- In Jupiter Notebook, maps are displayed directly, and the generated maps are also saved as HTML files.

## Code structure
The program consists of the following main functions:
- **`compute_bbox`**: Calculate bounding box based on origin and destination.
- **`Download_osm_data`**: Download OSM data through the Overpass API.
- **`haversine`**: a function of calculating the distance between two points.
- **`parse_osm`**: Parse OSM data to create nodes and graphs.
- **`find_nearest_node`**: Find the nearest node at the given coordinates.
- **`heuristic_haversine`**: a heuristic function used by the A* and Greedy algorithms.
- **`a_star_search`**: Implement the A* algorithm.
- **`Greedy_best_first_search`**: Implement the Greedy Best-First Search algorithm.
- **`bfs_search`**: Implement the BFS algorithm.
- **`dfs_search`**: Implement the DFS algorithm.
- **`compute_path_cost`**: Calculate the total distance of the path.
- **`add_route_to_map`**: Adds a discovered path to the map.
- **`create_base_map`**: Create a default map.

## Precautions
- **OSM Data Download**: A network connection is required and may take some time to download, depending on the size of the data.
- **Standard setting**: The origin and destination coordinates must be entered in latitude and longitude format.
- **Select algorithm**: The path and total distance may vary depending on the selected algorithm.

## Preview
![preview image](imgs/preview1.png)

## Licenses
This project will be distributed under [MIT License](./LICENSE), please refer to the License Document for more information.