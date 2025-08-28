# navigatorx

Openstreetmap routing engine in go. This project uses Contraction Hierarchies to speed up shortest path queries by preprocessing the road network graph (adding many shortcut edges) and Bidirectional Dijsktra for shortest path queries. H3 is used as a nearest neighbor query. 


## Quick Start

#### Only Preprocessing

```
1. download the example openstreetmap pbf file at the following link : https://drive.google.com/file/d/1NExZWDv9xueyDhaLeaeW0oDbt85rtvz2/view?usp=sharing
Note: or you can also use another openstreetmap file with osm.pbf format (https://download.geofabrik.de/)
2.  put the download results into the root directory of this project
3.  go mod tidy &&  mkdir bin
4. go build -o ./bin/navigatorx-preprocessing ./cmd/preprocessing
5. ./bin/navigatorx-preprocessing
Note: to replace the openstreetmap file, see the instructions below
(Minimum RAM 2.0 GB for the  openstreetmap data above)
5.  wait for preprocessing contraction hierarchies to complete (about 3 minutes)
Note: if error "resource temporary unavailable" -> just delete navigatorXDB directory and restart the steps above
```

### Only Server

Make sure you have done the preprocessing stage above!

```
1. go build -o ./bin/navigatorx-engine ./cmd/engine
2. ./bin/navigatorx-engine
```


#### Change Map Data

```
1. provide openstreetmap filename flag when running the program
 ./bin/navigatorx -f jakarta.osm.pbf
```

### Shortest Path Between 2 Place in Openstreetmap

```
1. wait until there is a log "server started at :5000".
2. request ke shortest path (source=surakarta , destination=fmipa ugm) [untuk file openstreetmap pbf pada step setup]
curl --location 'http://localhost:5000/api/navigations/shortest-path?src_lat=-7.567496&src_lon=110.812011&dst_lat=-7.767794&dst_lon=110.376511'
Note: Source & Destination Coordinates must be around Yogyakarta Province/Surakarta City/Klaten if using OpenStreetMap pb file in the setup step
5. Copy the polyline string path of the response endpoint result to https://valhalla.github.io/demos/polyline . Check Unsescape '\'. The shortest route will appear on the map. :)
```

### Alternative Routes
```
1.  wait until there is a log "server started at :5000".
curl --location 'http://localhost:5000/api/navigations/shortest-path-alternative-routes?src_lat=-7.75596&src_lon=110.37666&dst_lat=-7.554324&dst_lon=110.827275'

example 2:

curl --location 'http://localhost:5000/api/navigations/shortest-path-alternative-routes?src_lat=-7.75596&src_lon=110.37666&dst_lat=-7.802027937861759&dst_lon=110.39751911597698'
```


### Map Matching
```
0. download example map: https://drive.google.com/file/d/1AVY-o3fVDiWFvfr56T2pKmtKzr5t_rpX/view?usp=sharing
1. ./bin/navigatorx-preprocessing  -f washington.osm.pbf -mapmatch=true
NOTES: If you want to use the shortest path feature, you have to re-do the preprocessing with the mapmatch=false flag.
2. go build -o ./bin/navigatorx-mapmatch ./cmd/mapmatch
3. ./bin/navigatorx-mapmatch
4. open simple-web/mapmatch.html & upload data/msft-mapmatching.json
5. wait for map matching complete
```


### Traveling Salesman Problem 

What is the shortest (suboptimal) route to visit UGM, UNY, UPNV Jogja, UII Jogja, IAIN Surakarta, UNS, UMS, and ISI Surakarta campuses exactly once and return to the original campus location?

```
1. Wait until Contraction Hierarchies preprocessing is complete
2. request query traveling salesman problem
curl --location 'http://localhost:5000/api/navigations/tsp' \
--header 'Content-Type: application/json' \
--data '{
    "cities_coord": [
        {
            "lat": -7.773700556142326,
            "lon": 110.37927594982729
        },
        {
            "lat": -7.687798280189743,
            "lon": 110.41397147030537
        },
        {
            "lat": -7.773714842796234,
            "lon": 110.38625612460329
        },
        {
            "lat": -7.7620859704046135,
            "lon": 110.40928883503045
        },
        {
            "lat": -7.559256385020671,
            "lon":  110.85624887436603
        },
        {
            "lat": -7.558529640984029,
            "lon": 110.73442218529993
        },
        {
            "lat": -7.5579561088085665,
            "lon":  110.85233572375333
        },
        {
            "lat":  -7.557649260722883,
            "lon": 110.77068956586514
        }
    ]
}'

or (use ant-colony optimization)

curl --location 'http://localhost:5000/api/navigations/tsp_aco' \
--header 'Content-Type: application/json' \
--data '{
    "cities_coord": [
        {
            "lat": -7.773700556142326,
            "lon": 110.37927594982729
        },
        {
            "lat": -7.687798280189743,
            "lon": 110.41397147030537
        },
        {
            "lat": -7.773714842796234,
            "lon": 110.38625612460329
        },
        {
            "lat": -7.7620859704046135,
            "lon": 110.40928883503045
        },
        {
            "lat": -7.559256385020671,
            "lon":  110.85624887436603
        },
        {
            "lat": -7.558529640984029,
            "lon": 110.73442218529993
        },
        {
            "lat": -7.5579561088085665,
            "lon":  110.85233572375333
        },
        {
            "lat":  -7.557649260722883,
            "lon": 110.77068956586514
        }
    ]
}'
Note:  "cities_coord" must be a place around the province of Yogyakarta/Surakarta/Klaten if using OpenStreetMap data in the setup step
3.  Copy the polyline string path of the response endpoint result to https://valhalla.github.io/demos/polyline . Check Unsescape '\'. The shortest (suboptimal) TSP route will be displayed on the map. :)
```

### Rider-Driver Matchmaking 

```
1. Wait until Contraction Hierarchies preprocessing is complete
2. request to matchmaking api
curl --location 'http://localhost:5000/api/navigations/matching' \
--header 'Content-Type: application/json' \
--data '{
    "rider_lat_lon": [
        {
            "username": "rider1",
            "coord": {
                "lat": -7.767684016779731,
                "lon":  110.37649557875707
            }
        },
        {
            "username": "rider2",
            "coord": {
                "lat": -7.770534977253453,
                "lon":   110.38156022914536
            }
        },
        {
        "username":  "rider3",
        "coord": {
                "lat": -7.758553228167311,
                "lon":  110.39946726179075
            }
        },
        {
            "username": "rider4",
            "coord": {
                "lat": -7.801196956754633,
                "lon":  110.36672004587915
            }
        },
        {
            "username": "rider5",
            "coord": {
                "lat": -7.687706141646555,
                "lon": 110.41843469922163
            }
        },
       {
        "username": "rider6",
        "coord": {
                "lat": -7.556714132377571,
                "lon":  110.80520610633097
            }
       },
       {
        "username": "ridersolo7",
       "coord": {
            "lat": -7.561717618835495,
            "lon": 110.80992968611694
        }
       },
       {
         "username":  "rider8",
        "coord": {
                "lat": -7.5603675519267055,
                "lon": 110.76770911286172
            }
         },
         {
         "username":"rider9",
         "coord": {
            "lat": -7.740690926169796,
            "lon": 110.37411440130444
         }
        },
        {
            "username": "rider10",
            "coord": {
                "lat": -7.559722706161821,
                "lon": 110.85641658202763
            }
        } ,
        {
            "username":"rider11",
            "coord": {
                "lat": -7.516093248544381,
                "lon": 110.75452445432569
            }
        },
        {
            "username": "riderSolo",
            "coord": {
                "lat": -7.554605287475889,
                "lon": 110.82704286671313
            }
        },
        {
            "username": "riderSolo2",
            "coord": {
                "lat": -7.572505106627924,
                "lon": 110.84027742738219
            }
        }
    ],
     "driver_lat_lon": [
        {
            "username":    "driversolo3",
            "coord": {
                "lat": -7.573553087300021,
                "lon": 110.82073100556183
            }
        },
        {
            "username": "driver2",
            "coord": {
                "lat": -7.571130061786068,
            "lon":  110.80391906353825
            }
        },
        {
            "username":"driver3",
            "coord":{
                "lat": -7.782514997952533,
                "lon": 110.36659498380173
            }
        },
        {
            "username":  "driver4",
            "coord": {
                "lat": -7.781478644687624,
                "lon":  110.37267965620099
            }
        },
        {
            "username": "driver5",
            "coord": {
                "lat": -7.772515329567074,
                "lon": 110.37239634189628
            }
        },
        {
            "username": "driver6",
            "coord": {
                "lat": -7.755970087727186,
                "lon": 110.37634415191656
            }
        },
        {
            "username":   "driver7",
            "coord": {
                "lat": -7.764707027042284,
                "lon":  110.39259158173417
            }
        },
        {
            "username":  "driver8",
            "coord": {
                "lat": -7.565565153230303,
                "lon": 110.8079927641968
            }
        },
        {
            "username":  "driver9",
            "coord": {
            "lat": -7.751209845539939,
             "lon": 110.41778895149984
            }
        },
        {
            "username": "driverSolo",
            "coord": {
                "lat": -7.565093613983397,
                "lon": 110.81882158435778
            }
        },
        {
            "username": "driverSolo2",
            "coord": {
                "lat": -7.572505106632021,
                "lon":110.83008101439083
            }
        }


    ]
}'

3. The response is the most optimal rider-driver pairs based on proximity (estimated arrival time from driver to rider). Solved using the Hungarian algorithm.
```

### Many to Many Shortest Path Query

```
1. wait until preprocessing contraction hierarchies is complete
2. request  query many to many
curl --location 'http://localhost:5000/api/navigations/many-to-many' \
--header 'Content-Type: application/json' \
--data '{
    "sources": [{
        "lat": -7.550248257898637,
        "lon": 110.78217903249168
    },
    {
        "lat": -7.560347382387681,
        "lon": 110.78879587509478
    },
    {
        "lat": -7.5623445763181945,
        "lon": 110.81010426983109
    }
    ],
    "targets": [{
        "lat": -7.553672205152498,
        "lon": 110.79784256968716
    },
    {
        "lat": -7.564559782091322,
        "lon":  110.80455609811008
    },
    {
        "lat": -7.570135257838102,
        "lon": 110.82292649269334
    },
    {
        "lat": -7.598393719179397,
        "lon": 110.81555588473815
    }


    ]
}'

Note:  "sources" and "targets" must be around the province of Yogyakarta/Surakarta/Klaten if using OpenStreetMap data in the setup step
3.  Copy the polyline string path of the response endpoint result to https://valhalla.github.io/demos/polyline . Centang Unsescape '\'. Check Unsescape '\'. The shortest route of many to many query will be displayed on the map. :)
```

### Shortest Path with alternative street

```
1. wait until there is a log "server started at :5000".
2. request query shortest path w/ alternative street
curl --location 'http://localhost:5000/api/navigations/shortest-path-alternative-street?src_lat=-7.550261232598317&src_lon=110.78210790296636&street_alternative_lat=-7.8409667827395815&street_alternative_lon=110.3472473375829&dst_lat=-8.024431446370416&dst_lon=110.32971396395838'

Note:  "sources" and "targets" must be around the province of Yogyakarta/Surakarta/Klaten if using OpenStreetMap data in the setup step
3. Copy the polyline string path of the response endpoint result to https://valhalla.github.io/demos/polyline . Check Unsescape '\'. The shortest route will appear on the map. :)
```


## Profiling 

```
1.  curl "http://localhost:5000/debug/pprof/profile?seconds=30" -o cpu.pprof
2. open https://www.speedscope.app/ , upload cpu.pprof
```

####  Ref

```
-  R. Geisberger, P. Sanders, D. Schultes, and D. Delling, “Contraction Hierarchies: Faster and Simpler Hierarchical Routing in Road Networks,” in Experimental Algorithms, C. C. McGeoch, Ed., Berlin, Heidelberg: Springer, 2008, pp. 319–333. doi: 10.1007/978-3-540-68552-4_24.
- I. Abraham, D. Delling, A. V. Goldberg, and R. F. Werneck, “Alternative Routes in Road Networks”.
-   “Hidden Markov Map Matching Through Noise and Sparseness - Microsoft Research.” Accessed: Oct. 24, 2024. [Online]. Available: https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/
-  “Ant colony optimization | IEEE Journals & Magazine | IEEE Xplore.” Accessed: Dec. 08, 2024. [Online]. Available: https://ieeexplore-ieee-org.ezproxy.ugm.ac.id/document/4129846
-  https://en.wikipedia.org/wiki/Hungarian_algorithm
-  https://en.wikipedia.org/wiki/Simulated_annealing#:~:text=Simulated%20annealing%20(SA)%20is%20a,can%20find%20the%20global%20optimum.
-  https://jlazarsfeld.github.io/ch.150.project/sections/7-ch-overview/
```

