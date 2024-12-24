package kv

import (
	"fmt"
	"lintang/navigatorx/pkg/concurrent"
	"lintang/navigatorx/pkg/datastructure"
	"log"
	"math"

	"github.com/cockroachdb/pebble"
	"github.com/k0kubun/go-ansi"
	"github.com/schollz/progressbar/v3"
	"github.com/uber/h3-go/v4"
)

type KVDB struct {
	db *pebble.DB
}

func NewKVDB(db *pebble.DB) *KVDB {
	return &KVDB{db}
}

func (k *KVDB) CreateStreetKV(ways, hmmWays []datastructure.SurakartaWay, nodeIDxMap map[int64]int32, listenAddr string, isPreprocess bool) {
	fmt.Println("\nwait until loading contracted graph complete...")
	bar := progressbar.NewOptions(len(ways),
		progressbar.OptionSetWriter(ansi.NewAnsiStdout()),
		progressbar.OptionEnableColorCodes(true),
		progressbar.OptionShowBytes(true),
		progressbar.OptionSetWidth(15),
		progressbar.OptionSetDescription("[cyan][4/7][reset] Membuat h3 index untuk osm street..."),
		progressbar.OptionSetTheme(progressbar.Theme{
			Saucer:        "[green]=[reset]",
			SaucerHead:    "[green]>[reset]",
			SaucerPadding: " ",
			BarStart:      "[",
			BarEnd:        "]",
		}))
	kv := make(map[string][]datastructure.SmallWay)
	for i, w := range ways {
		street := ways[i]
		centerWayLat := w.CenterLoc[0]
		centerWayLon := w.CenterLoc[1]
		if len(street.IntersectionNodesID) == 0 {
			continue
		}
		h3LatLon := h3.NewLatLng(centerWayLat, centerWayLon)
		cell := h3.LatLngToCell(h3LatLon, 9)
		smallStreet := datastructure.SmallWay{
			CenterLoc:           []float64{centerWayLat, centerWayLon},
			IntersectionNodesID: street.IntersectionNodesID,
		}

		kv[cell.String()] = append(kv[cell.String()], smallStreet)

		bar.Add(1)
	}

	fmt.Println("")
	bar = progressbar.NewOptions(len(kv),
		progressbar.OptionSetWriter(ansi.NewAnsiStdout()),
		progressbar.OptionEnableColorCodes(true),
		progressbar.OptionShowBytes(true),
		progressbar.OptionSetWidth(15),
		progressbar.OptionSetDescription("[cyan][5/7][reset] saving h3 indexed street to pebble db..."),
		progressbar.OptionSetTheme(progressbar.Theme{
			Saucer:        "[green]=[reset]",
			SaucerHead:    "[green]>[reset]",
			SaucerPadding: " ",
			BarStart:      "[",
			BarEnd:        "]",
		}))

	workers := concurrent.NewWorkerPool[concurrent.SaveWayJobItem, interface{}](10, len(kv))

	for keyStr, valArr := range kv {
		conWay := make([]concurrent.SmallWay, len(valArr))
		for j, val := range valArr {
			conWay[j] = val.ToConcurrentWay()
		}

		workers.AddJob(concurrent.SaveWayJobItem{keyStr, conWay})
		bar.Add(1)
	}
	workers.Close()

	workers.Start(k.SaveWay)
	workers.Wait()

	fmt.Println("")
	k.SaveStreetHMMKV(hmmWays)
}

func (k *KVDB) SaveStreetHMMKV(hmmWays []datastructure.SurakartaWay) {
	kvHMM := make(map[string][]datastructure.SmallWay)
	for i, w := range hmmWays {
		street := hmmWays[i]
		centerWayLat := w.CenterLoc[0]
		centerWayLon := w.CenterLoc[1]

		h3LatLon := h3.NewLatLng(centerWayLat, centerWayLon)
		cell := h3.LatLngToCell(h3LatLon, 9)
		smallStreet := datastructure.SmallWay{
			CenterLoc:      []float64{centerWayLat, centerWayLon},
			NodesInBetween: street.Nodes,
			WayID:          street.WayID,
		}

		kvHMM[cell.String()+"-HMM"] = append(kvHMM[cell.String()+"-HMM"], smallStreet)
	}

	barHMM := progressbar.NewOptions(len(kvHMM),
		progressbar.OptionSetWriter(ansi.NewAnsiStdout()),
		progressbar.OptionEnableColorCodes(true),
		progressbar.OptionShowBytes(true),
		progressbar.OptionSetWidth(15),
		progressbar.OptionSetDescription("[cyan][5/7][reset] saving h3 indexed street for map matching to pebble db..."),
		progressbar.OptionSetTheme(progressbar.Theme{
			Saucer:        "[green]=[reset]",
			SaucerHead:    "[green]>[reset]",
			SaucerPadding: " ",
			BarStart:      "[",
			BarEnd:        "]",
		}))

	// save HMM ways
	workersHmm := concurrent.NewWorkerPool[concurrent.SaveWayJobItem, interface{}](10, len(kvHMM))

	for keyStr, valArr := range kvHMM {
		conWay := make([]concurrent.SmallWay, len(valArr))
		for j, val := range valArr {
			conWay[j] = val.ToConcurrentWay()
		}

		workersHmm.AddJob(concurrent.SaveWayJobItem{keyStr, conWay})
		barHMM.Add(1)
	}
	workersHmm.Close()
	workersHmm.Start(k.SaveWay)
	workersHmm.Wait()
	fmt.Println("")
}

func (k *KVDB) SaveWay(wayItem concurrent.SaveWayJobItem) interface{} {
	keyStr := wayItem.KeyStr
	valArr := wayItem.ValArr
	key := []byte(keyStr)
	ways := make([]SmallWay, len(valArr))
	for i, val := range valArr {
		nodesInBetweenLats := []float64{}
		nodesInBetweenLons := []float64{}
		for _, node := range val.NodesInBetween {
			nodesInBetweenLats = append(nodesInBetweenLats, node.Lat)
			nodesInBetweenLons = append(nodesInBetweenLons, node.Lon)
		}
		ways[i] = SmallWay{
			CenterLoc:           val.CenterLoc,
			IntersectionNodesID: val.IntersectionNodesID,
			NodesInBetween:      NewCoordinates(nodesInBetweenLats, nodesInBetweenLons),
			WayID:               val.WayID,
		}
	}

	val, err := EncodeWay(ways)

	if err != nil {
		log.Fatal(err)
	}
	if err := k.db.Set(key, val, pebble.Sync); err != nil {
		log.Fatal(err)
	}
	return nil
}

// GetNearestStreetsFromPointCoord buat road snaping. Untuk menentukan jalan-jalan yang paling dekat dengan titik start/end rute yang di tunjuk sama pengguna
func (k *KVDB) GetNearestStreetsFromPointCoord(lat, lon float64, hmm bool) ([]datastructure.SmallWay, error) {
	ways := []SmallWay{}

	home := h3.NewLatLng(lat, lon)
	cell := h3.LatLngToCell(home, 9)
	suffixHMM := ""
	if hmm {
		suffixHMM = "-HMM"
	}

	cellString := cell.String() + suffixHMM
	val, closer, err := k.db.Get([]byte(cellString))
	if err == nil {
		defer closer.Close()
	}

	streets, _ := LoadWay(val)

	ways = append(ways, streets...)

	cells := kRingIndexesArea(lat, lon, 1) // search cell neighbor dari homeCell yang radius nya 1 km
	if len(ways) == 0 {
		for _, currCell := range cells {
			if currCell == cell {
				continue
			}
			currCellString := currCell.String() + suffixHMM
			val, closer, err := k.db.Get([]byte(currCellString))
			if closer == nil || err != nil {
				continue
			}

			streets, err := LoadWay(val)
			if err != nil {
				closer.Close()
				return []datastructure.SmallWay{}, err
			}
			ways = append(ways, streets...)
			closer.Close()
		}
	}

	// kalau dari radius 1 km dari titik start/end rute pengguna gak ada jalan (misal di bandara, hutan, dll). maka cari jalan dari neighbor h3 cell yang lebih jauh dari titik start/end rute
	for lev := 1; lev <= 10; lev++ {
		if len(ways) == 0 {
			cells := h3.GridDisk(cell, lev)
			for _, currCell := range cells {
				if currCell == cell {
					continue
				}
				currCellString := currCell.String() + suffixHMM
				val, closer, err := k.db.Get([]byte(currCellString))
				if closer == nil || err != nil {
					continue
				}

				streets, err := LoadWay(val)
				if err != nil {
					closer.Close()
					return []datastructure.SmallWay{}, err
				}
				ways = append(ways, streets...)
				closer.Close()
			}
		} else {
			break
		}
	}

	if len(ways) == 0 {

		return []datastructure.SmallWay{}, fmt.Errorf("tidak ada jalan di sekitar lokasi")
	}
	waysData := make([]datastructure.SmallWay, len(ways))
	for i, way := range ways {
		nodeInBetweenLats := []float64{}
		nodeInBetweenLons := []float64{}
		for _, node := range way.NodesInBetween {
			nodeInBetweenLats = append(nodeInBetweenLats, node.Lat)
			nodeInBetweenLons = append(nodeInBetweenLons, node.Lon)
		}
		coords := datastructure.NewCoordinates(nodeInBetweenLats, nodeInBetweenLons)
		waysData[i] = datastructure.SmallWay{
			CenterLoc:           way.CenterLoc,
			IntersectionNodesID: way.IntersectionNodesID,
			NodesInBetween:      coords,
			WayID:               way.WayID,
		}
	}

	return waysData, nil
}

/*
*
  - https://observablehq.com/@nrabinowitz/h3-radius-lookup?collection=@nrabinowitz/h3
    search cell neighbor dari cell dari lat,lon  yang radius nya = searchRadiusKm
*/
func kRingIndexesArea(lat, lon, searchRadiusKm float64) []h3.Cell {
	home := h3.NewLatLng(lat, lon)
	origin := h3.LatLngToCell(home, 9)
	originArea := h3.CellAreaKm2(origin)
	searchArea := math.Pi * searchRadiusKm * searchRadiusKm

	radius := 0
	diskArea := originArea

	for diskArea < searchArea {
		radius++
		cellCount := float64(3*radius*(radius+1) + 1)
		diskArea = cellCount * originArea
	}

	return h3.GridDisk(origin, radius)
}

func (k *KVDB) Close() {
	k.db.Close()
}
