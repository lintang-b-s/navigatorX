package kv

import (
	"context"
	"errors"
	"fmt"
	"log"
	"math"
	"sync"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"
	"go.etcd.io/bbolt"

	"github.com/uber/h3-go/v4"
)

const (
	H3_BOLTBUCKET = "h3-bucket"
)

var (
	ErrEdgesNotFound = errors.New("edges not found")
)

type KVDB struct {
	db *bbolt.DB
	sync.Mutex
}

func NewKVDB(db *bbolt.DB) *KVDB {
	return &KVDB{db, sync.Mutex{}}
}

func (k *KVDB) BuildH3IndexedEdges(ctx context.Context, graphStorage *datastructure.GraphStorage) error {
	log.Printf("wait until loading contracted graph complete...")

	log.Printf("creating & saving h3 indexed road segments to key-value db...")
	kv := make(map[string][]datastructure.KVEdge)
	for i := range graphStorage.EdgeStorage {
		select {
		case <-ctx.Done():
			return fmt.Errorf("context cancelled")
		default:
		}

		roadSegment := graphStorage.EdgeStorage[i]
		startIndex := graphStorage.MapEdgeInfo[i].StartPointsIndex
		endIndex := graphStorage.MapEdgeInfo[i].EndPointsIndex
		var (
			pointsInBetween []datastructure.Coordinate
		)
		if startIndex < endIndex {
			pointsInBetween = graphStorage.GlobalPoints[graphStorage.MapEdgeInfo[i].StartPointsIndex:graphStorage.MapEdgeInfo[i].EndPointsIndex]
		} else {
			for j := startIndex - 1; j >= endIndex; j-- {
				pointsInBetween = append(pointsInBetween, graphStorage.GlobalPoints[j])
			}
		}

		edgeLat := pointsInBetween[0].Lat
		edgeLon := pointsInBetween[0].Lon

		h3LatLon := h3.NewLatLng(edgeLat, edgeLon)
		cell := h3.LatLngToCell(h3LatLon, h3CellLevel)
		smallSegment := datastructure.KVEdge{
			CenterLoc:  [2]float64{edgeLat, edgeLon},
			FromNodeID: roadSegment.FromNodeID,
			ToNodeID:   roadSegment.ToNodeID,
		}

		kv[cell.String()] = append(kv[cell.String()], smallSegment)

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

	log.Printf("creating & saving h3 indexed street segment to key-value db done...")
	return nil
}

type batchData struct {
	key   string
	value []datastructure.KVEdge
}

func (k *KVDB) saveBatchEdges(ctx context.Context, batchData []batchData) error {
	k.Lock()
	defer k.Unlock()
	return k.db.Batch(func(tx *bbolt.Tx) error {
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

			bucket := tx.Bucket([]byte(H3_BOLTBUCKET))
			if err := bucket.Put(key, val); err != nil {
				return err
			}
		}
		return nil
	})

}

func (k *KVDB) get(key []byte) (node []byte, err error) {
	k.db.View(func(tx *bbolt.Tx) error {
		bucket := tx.Bucket([]byte(H3_BOLTBUCKET))
		node = bucket.Get([]byte(key))
		if node == nil {
			return ErrEdgesNotFound
		}
		return nil
	})
	return
}

const (
	searchRadiusInKM = 0.15
	h3CellLevel      = 9
)

func (k *KVDB) GetNearestRoadSegmentsFromPointCoord(lat, lon float64) ([]datastructure.KVEdge, error) {
	edges := []datastructure.KVEdge{}

	var (
		err error
	)

	edges, err = k.radiusSearch(lat, lon, searchRadiusInKM)
	if err != nil && !errors.Is(err, ErrEdgesNotFound) {
		return []datastructure.KVEdge{}, err
	}

	radius := searchRadiusInKM

	for len(edges) == 0 && radius < 1.0 {
		radius += 0.05
		edges, err = k.radiusSearch(lat, lon, radius)
		if err != nil && !errors.Is(err, ErrEdgesNotFound) {
			return []datastructure.KVEdge{}, err
		}
	}

	if len(edges) == 0 {

		return []datastructure.KVEdge{}, ErrEdgesNotFound
	}

	return edges, nil
}

func (k *KVDB) radiusSearch(lat, lon float64, radius float64) ([]datastructure.KVEdge, error) {
	var (
		edges    []datastructure.KVEdge
		segments []datastructure.KVEdge
	)

	cells := kRingIndexesArea(lat, lon, radius)
	for _, currCell := range cells {

		currCellString := currCell.String()

		val, err := k.get([]byte(currCellString))

		if err != nil && !errors.Is(err, ErrEdgesNotFound) {
			return []datastructure.KVEdge{}, err
		}

		if val != nil {
			segments, err = loadEdges(val)
			if err != nil {
				return []datastructure.KVEdge{}, err
			}
		}
		edges = append(edges, segments...)
	}
	return edges, nil
}

func kRingIndexesArea(lat, lon, searchRadiusKm float64) []h3.Cell {
	home := h3.NewLatLng(lat, lon)
	origin := h3.LatLngToCell(home, h3CellLevel)
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
