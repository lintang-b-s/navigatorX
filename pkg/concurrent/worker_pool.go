package concurrent

import (
	"sync"
)

type WorkerPool[T JobI, G any] struct {
	numWorkers int
	jobQueue   chan T
	results    chan G
	wg         sync.WaitGroup
}

func NewWorkerPool[T JobI, G any](numWorkers, jobQueueSize int) *WorkerPool[T, G] {
	return &WorkerPool[T, G]{
		numWorkers: numWorkers,
		jobQueue:   make(chan T, jobQueueSize),
		results:    make(chan G, jobQueueSize),
	}
}

func (wp *WorkerPool[JobI, G]) worker(id int, jobFunc JobFunc[JobI, G]) {
	defer wp.wg.Done()
	for job := range wp.jobQueue {
		res := jobFunc(job)
		wp.results <- res
	}
}

func (wp *WorkerPool[JobI, G]) Start(jobFunc JobFunc[JobI, G]) {
	for i := 1; i <= wp.numWorkers; i++ {
		wp.wg.Add(1)
		go wp.worker(i, jobFunc)
	}
}

func (wp *WorkerPool[JobI, G]) Wait() {
	go func() {
		wp.wg.Wait()
		close(wp.results)
	}()
}

func (wp *WorkerPool[JobI, G]) AddJob(job JobI) {
	wp.jobQueue <- job
}

func (wp *WorkerPool[JobI, G]) CollectResults() chan G {
	return wp.results
}

func (wp *WorkerPool[JobI, G]) Close() {
	close(wp.jobQueue)
}
