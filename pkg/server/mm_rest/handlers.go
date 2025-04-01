package mmrest

import (
	"context"
	"errors"
	"fmt"
	"net/http"

	"github.com/lintang-b-s/navigatorx/pkg/datastructure"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/render"
	"github.com/go-playground/locales/en"
	ut "github.com/go-playground/universal-translator"
	"github.com/go-playground/validator/v10"
	enTranslations "github.com/go-playground/validator/v10/translations/en"
)

type MapMatchingService interface {
	MapMatch(ctx context.Context, gps []datastructure.Coordinate) (string,
		[]datastructure.Coordinate, []datastructure.EdgeCH, []datastructure.Coordinate, error)
	NearestRoadSegments(ctx context.Context, lat, lon float64, radius float64, k int) ([]datastructure.EdgeCH, []float64, error)
}

type MapMatchingHandler struct {
	svc MapMatchingService
}

func MapMatchingRouter(r *chi.Mux, svc MapMatchingService) {
	handler := &MapMatchingHandler{svc}

	r.Group(func(r chi.Router) {
		r.Route("/api/map-match", func(r chi.Router) {
			r.Post("/map-matching", handler.MapMatch)
			r.Post("/nearest-road-segment", handler.NearestRoadSegments)

		})
	})
}

// MapMatchingRequest model info
//
//	@Description	request body untuk map matching pakai hidden markov model
type MapMatchingRequest struct {
	Coordinates []Coord `json:"coordinates" validate:"required,dive"`
}

// Coord model info
//
//	@Description	model untuk koordinat
type Coord struct {
	Lat float64 `json:"lat" validate:"required,lt=90,gt=-90"`
	Lon float64 `json:"lon" validate:"required,lt=180,gt=-180"`
}

func (s *MapMatchingRequest) Bind(r *http.Request) error {
	if len(s.Coordinates) == 0 {
		return errors.New("invalid request")
	}
	return nil
}

// MapMatchingResponse model info
//
//	@Description	response body untuk map matching pakai hidden markov model
type MapMatchingResponse struct {
	Path  string `json:"path"`
	Snaps []struct {
		Coord       Coord                `json:"coordinates"`
		Observation Coord                `json:"observation"`
		Edge        datastructure.EdgeCH `json:"edge"`
	} `json:"snaps,omitempty"`
}

func RenderMapMatchingResponse(path string, coords []datastructure.Coordinate, edges []datastructure.EdgeCH, obsPath []datastructure.Coordinate) *MapMatchingResponse {
	snapsResp := []struct {
		Coord       Coord `json:"coordinates"`
		Observation Coord `json:"observation"`

		Edge datastructure.EdgeCH `json:"edge"`
	}{}
	for i, c := range coords {
		snapsResp = append(snapsResp, struct {
			Coord       Coord "json:\"coordinates\""
			Observation Coord "json:\"observation\""

			Edge datastructure.EdgeCH "json:\"edge\""
		}{
			Coord{
				Lat: c.Lat,
				Lon: c.Lon,
			},
			Coord{
				Lat: obsPath[i].Lat,
				Lon: obsPath[i].Lon,
			},
			edges[i],
		})
	}

	return &MapMatchingResponse{
		Path:  path,
		Snaps: snapsResp,
	}
}

// MapMatch
//
//	@Summary		map matching pakai hidden markov model. Snapping noisy GPS coordinates ke road network lokasi asal gps seharusnya
//	@Description	map matching pakai hidden markov model. Snapping noisy GPS coordinates ke road network lokasi asal gps seharusnya
//	@Tags			navigations
//	@Param			body	body	MapMatchingRequest	true	"request body hidden markov model map matching"
//	@Accept			application/json
//	@Produce		application/json
//	@Router			/navigations/map-matching [post]
//	@Success		200	{object}	MapMatchingResponse
//	@Failure		400	{object}	ErrResponse
//	@Failure		500	{object}	ErrResponse
func (h *MapMatchingHandler) MapMatch(w http.ResponseWriter, r *http.Request) {
	data := &MapMatchingRequest{}
	if err := render.Bind(r, data); err != nil {
		render.Render(w, r, ErrInvalidRequest(err))
		return
	}
	validate := validator.New()
	if err := validate.Struct(*data); err != nil {
		english := en.New()
		uni := ut.New(english, english)
		trans, _ := uni.GetTranslator("en")
		_ = enTranslations.RegisterDefaultTranslations(validate, trans)
		vv := translateError(err, trans)
		render.Render(w, r, ErrValidation(err, vv))
		return
	}

	coords := []datastructure.Coordinate{}
	for _, c := range data.Coordinates {
		coords = append(coords, datastructure.Coordinate{
			Lat: c.Lat,
			Lon: c.Lon,
		})
	}
	p, pNode, edges, obsPath, err := h.svc.MapMatch(r.Context(), coords)
	if err != nil {
		render.Render(w, r, ErrInternalServerErrorRend(errors.New("internal server error")))

		return
	}

	render.Status(r, http.StatusOK)
	render.JSON(w, r, RenderMapMatchingResponse(p, pNode, edges, obsPath))
}

// RoadSnappingRequest model info
//
//	@Description	request body roadsnapping
type RoadSnappingRequest struct {
	Lat    float64 `json:"lat"`
	Lon    float64 `json:"lon"`
	Radius float64 `json:"radius"`
	K      int     `json:"k"`
}

func (s *RoadSnappingRequest) Bind(r *http.Request) error {
	if s.Radius == 0 || s.K == 0 {
		return errors.New("invalid request")
	}
	return nil
}

// RoadSnappingResponse model info
//
//	@Description	response body for road snapping
type RoadSnappingResponse struct {
	Edges []struct {
		Edge     datastructure.EdgeCH `json:"edge"`
		Distance float64              `json:"distance"`
	} `json:"edges"`
}

func RenderRoadSnappingResponse(edges []datastructure.EdgeCH, dists []float64) *RoadSnappingResponse {
	edgesResp := make([]struct {
		Edge     datastructure.EdgeCH `json:"edge"`
		Distance float64              `json:"distance"`
	}, 0)
	for i, e := range edges {
		edgesResp = append(edgesResp, struct {
			Edge     datastructure.EdgeCH `json:"edge"`
			Distance float64              `json:"distance"`
		}{
			Edge:     e,
			Distance: dists[i],
		})
	}
	return &RoadSnappingResponse{
		Edges: edgesResp,
	}
}

func (h *MapMatchingHandler) NearestRoadSegments(w http.ResponseWriter, r *http.Request) {
	data := &RoadSnappingRequest{}
	if err := render.Bind(r, data); err != nil {
		render.Render(w, r, ErrInvalidRequest(err))
		return
	}
	validate := validator.New()
	if err := validate.Struct(*data); err != nil {
		english := en.New()
		uni := ut.New(english, english)
		trans, _ := uni.GetTranslator("en")
		_ = enTranslations.RegisterDefaultTranslations(validate, trans)
		vv := translateError(err, trans)
		render.Render(w, r, ErrValidation(err, vv))
		return
	}

	roadSegments, dists, err := h.svc.NearestRoadSegments(r.Context(), data.Lat, data.Lon, data.Radius, data.K)
	if err != nil {
		render.Render(w, r, ErrInternalServerErrorRend(errors.New("internal server error")))

		return
	}

	render.Status(r, http.StatusOK)
	render.JSON(w, r, RenderRoadSnappingResponse(roadSegments, dists))
}

func ErrInvalidRequest(err error) render.Renderer {
	return &ErrResponse{
		Err:            err,
		HTTPStatusCode: 400,
		StatusText:     "Invalid request.",
		ErrorText:      err.Error(),
	}
}

// ErrResponse model info
//
//	@Description	model untuk error response
type ErrResponse struct {
	Err            error `json:"-"` // low-level runtime error
	HTTPStatusCode int   `json:"-"` // http response status code

	StatusText    string   `json:"status"`          // user-level status message
	AppCode       int64    `json:"code,omitempty"`  // application-specific error code
	ErrorText     string   `json:"error,omitempty"` // application-level error message, for debugging
	ErrValidation []string `json:"validation,omitempty"`
}

func (e *ErrResponse) Render(w http.ResponseWriter, r *http.Request) error {
	render.Status(r, e.HTTPStatusCode)
	return nil
}

func translateError(err error, trans ut.Translator) (errs []error) {
	if err == nil {
		return nil
	}
	validatorErrs := err.(validator.ValidationErrors)
	for _, e := range validatorErrs {
		translatedErr := fmt.Errorf(e.Translate(trans))
		errs = append(errs, translatedErr)
	}
	return errs
}

func ErrValidation(err error, errV []error) render.Renderer {
	vv := []string{}
	for _, v := range errV {
		vv = append(vv, v.Error())
	}
	return &ErrResponse{
		Err:            err,
		HTTPStatusCode: 400,
		StatusText:     "Invalid request.",
		ErrorText:      err.Error(),
		ErrValidation:  vv,
	}
}

func ErrInternalServerErrorRend(err error) render.Renderer {
	return &ErrResponse{
		Err:            err,
		HTTPStatusCode: 500,
		StatusText:     "Internal server error.",
		ErrorText:      err.Error(),
	}
}
