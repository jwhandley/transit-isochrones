# Public transport isochrones

This is a web server that exposes a simple API for computing public transportation isochrones for a given area. It requires an [osm.pbf](https://wiki.openstreetmap.org/wiki/PBF_Format) and [gtfs file](https://gtfs.org/) in order to work. For a given start coordinates, arrival time, and travel duration it will provide a geojson of the area accessible using public transport.

## Quick start

You need to have Rust installed to run the server. You can run it using the following command.

```bash
cargo run --release -- [path/to/osm/file.osm.pbf] [path/to/gtfs.zip]
```

Once the server is running, you can send GET requests to receive isochrones.

```bash
curl localhost:8000/isochrone?lat=<latitude>&lon=<longitude>&arrival_time=<arrival_time>&duration=<duration>
```

## Todo

This project is still a work in progress. The main improvements I have in the pipeline are:
- Making the graph stucture more memory efficient
- Adding more modes of transport (i.e., walking and driving)
- Add geocoding functionality so users can supply location names instead of coordinates
- Building a basic front end for the web