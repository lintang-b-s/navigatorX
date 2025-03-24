package kv

import (
	"context"
	"errors"
	"fmt"
	"lintang/navigatorx/pkg/datastructure"
	"log"
	"math"

	"github.com/dgraph-io/badger/v4"
	"github.com/uber/h3-go/v4"
)

var (
	ErrEdgesNotFound = errors.New("edges not found")
)

type KVDB struct {
	db *badger.DB
}

func NewKVDB(db *badger.DB) *KVDB {
	return &KVDB{db}
}

func (k *KVDB) BuildH3IndexedEdges(ctx context.Context, edges []datastructure.EdgeCH, edgesExtraInfo []datastructure.EdgeExtraInfo) error {
	log.Printf("wait until loading contracted graph complete...")

	log.Printf("creating & saving h3 indexed street to key-value db...")
	kv := make(map[string][]datastructure.KVEdge)
	for i := range edges {
		select {
		case <-ctx.Done():
			return fmt.Errorf("context cancelled")
		default:
		}

		roadSegment := edges[i]

		pointsInBetween := edgesExtraInfo[i].PointsInBetween

		edgeLat := pointsInBetween[0].Lat
		edgeLon := pointsInBetween[0].Lon

		h3LatLon := h3.NewLatLng(edgeLat, edgeLon)
		cell := h3.LatLngToCell(h3LatLon, 9)
		smallStreet := datastructure.KVEdge{
			CenterLoc:  [2]float64{edgeLat, edgeLon},
			FromNodeID: roadSegment.FromNodeID,
			ToNodeID:   roadSegment.ToNodeID,
		}

		kv[cell.String()] = append(kv[cell.String()], smallStreet)

	}

	batchSize := 1000
	batches := make([]batchData, 0, batchSize)
	for key, value := range kv {
		select {
		case <-ctx.Done():
			return fmt.Errorf("context cancelled")
		default:
		}

		batches = append(batches, batchData{
			key:   key,
			value: value,
		})
		if len(batches) == batchSize {
			err := k.saveBatchEdges(ctx, batches)
			if err != nil {
				return err
			}
			batches = make([]batchData, 0, batchSize)
		}

	}

	if len(batches) > 0 {
		err := k.saveBatchEdges(ctx, batches)
		if err != nil {
			return err
		}
	}

	log.Printf("creating & saving h3 indexed street to key-value db done...")
	return nil
}

type batchData struct {
	key   string
	value []datastructure.KVEdge
}

func (k *KVDB) saveBatchEdges(ctx context.Context, batchData []batchData) error {
	batch := k.db.NewWriteBatch()
	defer batch.Cancel()

	for _, data := range batchData {
		select {
		case <-ctx.Done():
			return fmt.Errorf("context cancelled")
		default:
		}

		key := []byte(data.key)

		val, err := encodeEdges(data.value)

		if err != nil {
			return err
		}

		if err := batch.Set(key, val); err != nil {
			return err
		}
	}

	if err := batch.Flush(); err != nil {
		log.Printf("error saving edges: %v", err)
		return err
	}
	log.Printf("saving %d edges done", len(batchData))
	return nil
}

func (k *KVDB) get(val, key []byte) ([]byte, error) {
	err := k.db.View(func(txn *badger.Txn) error {
		item, err := txn.Get([]byte(key))
		if err != nil {
			return err
		}

		val, err = item.ValueCopy(nil)
		if err != nil {
			return err
		}

		return nil
	})
	return val, err
}

func (k *KVDB) GetNearestStreetsFromPointCoord(lat, lon float64) ([]datastructure.KVEdge, error) {
	edges := []datastructure.KVEdge{}

	home := h3.NewLatLng(lat, lon)
	cell := h3.LatLngToCell(home, 9)

	cellString := cell.String()

	var val []byte
	val, err := k.get(val, []byte(cellString))

	if err != nil {
		return []datastructure.KVEdge{}, err
	}

	streets, err := loadEdges(val)
	if err != nil {
		return []datastructure.KVEdge{}, err
	}

	edges = append(edges, streets...)

	cells := kRingIndexesArea(lat, lon, 1)
	if len(edges) == 0 {
		for _, currCell := range cells {
			if currCell == cell {
				continue
			}
			currCellString := currCell.String()

			var val []byte
			val, err = k.get(val, []byte(currCellString))

			if err != nil && !errors.Is(err, badger.ErrKeyNotFound) {
				return []datastructure.KVEdge{}, err
			}

			streets, err := loadEdges(val)
			if err != nil {
				return []datastructure.KVEdge{}, err
			}
			edges = append(edges, streets...)
		}
	}

	for lev := 1; lev <= 10; lev++ {
		if len(edges) == 0 {
			cells := h3.GridDisk(cell, lev)
			for _, currCell := range cells {
				if currCell == cell {
					continue
				}
				currCellString := currCell.String()
				var val []byte
				val, err = k.get(val, []byte(currCellString))

				if err != nil && !errors.Is(err, badger.ErrKeyNotFound) {
					return []datastructure.KVEdge{}, err
				}
				streets, err := loadEdges(val)
				if err != nil {
					return []datastructure.KVEdge{}, err
				}
				edges = append(edges, streets...)
			}
		} else {
			break
		}
	}

	if len(edges) == 0 {

		return []datastructure.KVEdge{}, ErrEdgesNotFound
	}

	return edges, nil
}

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
