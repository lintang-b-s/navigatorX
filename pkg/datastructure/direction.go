package datastructure

import (
	"fmt"
	"strings"

	"github.com/lintang-b-s/navigatorx/pkg/util"
)

const (
	UNKNOWN            = -9999
	U_TURN_UNKNOWN     = -999
	U_TURN_LEFT        = -8
	KEEP_LEFT          = -7
	LEAVE_ROUNDABOUT   = -6
	TURN_SHARP_LEFT    = -3
	TURN_LEFT          = -2
	TURN_SLIGHT_LEFT   = -1
	CONTINUE_ON_STREET = 0
	TURN_SLIGHT_RIGHT  = 1
	TURN_RIGHT         = 2
	TURN_SHARP_RIGHT   = 3
	FINISH             = 4
	USE_ROUNDABOUT     = 6
	IGNORE             = 9999999
	KEEP_RIGHT         = 7
	U_TURN_RIGHT       = 8
	START              = 101
)

type Instruction struct {
	Point              Coordinate
	RawName            bool
	Sign               int
	Name               string
	CumulativeDistance float64
	CumulativeEta      float64
	ExtraInfo          map[string]interface{}
	IsRoundabout       bool
	Roundabout         RoundaboutInstruction
	EdgeIDs            []int32 // biar pas di fe, bisa tau driver lagi di edge mana & bisa kasih lihat driving direction (edgeID dari driver bisa tau dari hasil realtime map matching dari current gps point driver)
	EdgeSpeed          float64 // biar pas di fe, bisa nentuin berapa eta ke titik belok dari current position driver
	Points             []Coordinate
	TurnBearing        float64 // final bearing dari prevEdge sebelum turn point. final bearing = bearing dari point prevEdge.from -> point prevEdge.to dengan meridian line crossing point prevEdge.to
	TurnType           string
}

func NewInstruction(sign int, name string, p Coordinate, isRoundAbout bool, edgeIDs []int32,
	cumulativeDist, cumulativeEta float64, points []Coordinate, turnBearing float64) Instruction {
	var roundabout RoundaboutInstruction
	var ins Instruction
	if isRoundAbout {
		roundabout = NewRoundaboutInstruction()
		ins = Instruction{
			Sign:               sign,
			Name:               name,
			Point:              p,
			ExtraInfo:          make(map[string]interface{}, 3),
			Roundabout:         roundabout,
			IsRoundabout:       true,
			CumulativeEta:      cumulativeEta,
			CumulativeDistance: cumulativeDist,
			EdgeIDs:            edgeIDs,
			Points:             points,
			TurnBearing:        turnBearing,
		}

	} else {
		ins = Instruction{
			Sign:               sign,
			Name:               name,
			Point:              p,
			ExtraInfo:          make(map[string]interface{}, 3),
			IsRoundabout:       false,
			CumulativeEta:      cumulativeEta,
			CumulativeDistance: cumulativeDist,
			EdgeIDs:            edgeIDs,
			Points:             points,
			TurnBearing:        turnBearing,
		}
	}

	_, ins.TurnType = getDirectionDescription(sign, ins)

	return ins
}

func NewInstructionWithRoundabout(sign int, name string, p Coordinate, isRoundAbout bool, roundabout RoundaboutInstruction,
	cumulativeDistance, cumulativeEta float64, edgeIDs []int32, turnBearing float64) Instruction {
	ins := Instruction{
		Sign:               sign,
		Name:               name,
		Point:              p,
		ExtraInfo:          make(map[string]interface{}, 3),
		Roundabout:         roundabout,
		IsRoundabout:       isRoundAbout,
		CumulativeDistance: cumulativeDistance,
		CumulativeEta:      cumulativeEta,
		EdgeIDs:            edgeIDs,
		TurnBearing:        turnBearing,
	}
	ins.TurnType = "ROUNDABOUT"
	return ins
}

func (instr *Instruction) GetName() string {
	if instr.Name == "" {
		if name, ok := instr.ExtraInfo["street_ref"].(string); ok {
			return name
		}
		return ""
	}
	return instr.Name
}

func (instr *Instruction) GetTurnDescription() string {
	if instr.RawName {
		return instr.Name
	}

	streetName := instr.GetName()
	sign := instr.Sign
	var description string

	switch sign {
	case CONTINUE_ON_STREET:
		if isEmpty(streetName) {
			description = "Continue"
		} else {
			description = fmt.Sprintf("Continue onto %s", streetName)
		}
	case START:
		if heading, ok := instr.ExtraInfo["heading"]; ok {
			headingAngle := heading.(float64)
			if headingAngle < 0.0 {
				headingAngle += 360
			}
			compassDir := bearingToCompass(headingAngle)
			description = fmt.Sprintf("Head %s toward %s", compassDir, streetName)
		} else {
			description = fmt.Sprintf("Head toward %s", streetName)
		}
	case FINISH:
		description = fmt.Sprint("you have arrived at your destination")
	default:
		dir, _ := getDirectionDescription(sign, *instr)
		if dir == "" {
			description = fmt.Sprintf("unknown  %d", sign)
		} else {
			if isEmpty(streetName) {
				description = dir
			} else {
				switch dir {
				case "Keep left":
					description = fmt.Sprintf("%s to continue on %s", dir, streetName)
				case "Keep right":
					description = fmt.Sprintf("%s continue on %s", dir, streetName)

				default:
					description = fmt.Sprintf("%s onto %s", dir, streetName)

				}
			}
		}
	}

	dest, _ := instr.ExtraInfo["street_destination"].(string)
	destRef, _ := instr.ExtraInfo["street_destination_ref"].(string)

	if dest != "" {
		if destRef != "" {
			return fmt.Sprintf("toward_destination_with_ref %s  %s %s", description, destRef, dest)
		}
		return fmt.Sprintf("toward_destination %s  %s", description, dest)
	} else if destRef != "" {
		return fmt.Sprintf("toward_destination_ref_only %s  %s", description, destRef)
	}

	return description
}
func bearingToCompass(bearing float64) string {
	if bearing < 22.5 {
		return "North"
	} else if bearing < 67.5 {
		return "North East"
	} else if bearing < 112.5 {
		return "East"
	} else if bearing < 157.5 {
		return "South East"
	} else if bearing < 202.5 {
		return "South"
	} else if bearing < 247.5 {
		return "South West"
	} else if bearing < 292.5 {
		return "West"
	} else if bearing < 337.5 {
		return "North West"
	} else {
		return "North"
	}
}
func getDirectionDescription(sign int, instruction Instruction) (string, string) {
	switch sign {
	case U_TURN_UNKNOWN:
		return "Make U-turn", "U_TURN_RIGHT"
	case U_TURN_RIGHT:
		return "Make U-turn right", "U_TURN_RIGHT"
	case U_TURN_LEFT:
		return "Make U-turn left", "U_TURN_LEFT"
	case KEEP_LEFT:
		return "Keep left", "KEEP_LEFT"
	case TURN_SHARP_LEFT:
		return "Turn sharp left", "TURN_SHARP_LEFT"
	case TURN_LEFT:
		return "Turn left", "TURN_LEFT"
	case TURN_SLIGHT_LEFT:
		return "Turn slight left", "TURN_SLIGHT_LEFT"
	case TURN_SLIGHT_RIGHT:
		return "Turn slight right", "TURN_SLIGHT_RIGHT"
	case TURN_RIGHT:
		return "Turn right", "TURN_RIGHT"
	case TURN_SHARP_RIGHT:
		return "Turn sharp right", "TURN_SHARP_RIGHT"
	case KEEP_RIGHT:
		return "Keep right", "KEEP_RIGHT"
	case USE_ROUNDABOUT:
		if !instruction.Roundabout.Exited {
			return "Enter the roundabout", "USE_ROUNDABOUT"
		}
		roundaboutDir := "clockwise" // bundaran  di indo selalu clockwise

		return fmt.Sprintf("At Roundabout, take the exit point %d %s", instruction.Roundabout.ExitNumber, roundaboutDir), ""

	default:
		return "", ""
	}
}

func isEmpty(str string) bool {
	return strings.TrimSpace(str) == ""
}

type RoundaboutInstruction struct {
	ExitNumber int
	Exited     bool
}

type Option func(RoundaboutInstruction) RoundaboutInstruction

func NewRoundaboutInstruction(options ...Option) RoundaboutInstruction {
	roundabout := RoundaboutInstruction{
		ExitNumber: 0,
		Exited:     false,
	}
	for _, option := range options {
		roundabout = option(roundabout)
	}
	return roundabout
}

type DrivingDirection struct {
	Instruction string     `json:"instruction"`
	Point       Coordinate `json:"turn_point"`
	StreetName  string     `json:"street_name"`
	ETA         float64    `json:"eta"`
	Distance    float64    `json:"distance"`
	EdgeIDs     []int32    `json:"edge_ids"`
	Polyline    string     `json:"polyline"`
	TurnBearing float64    `json:"turn_bearing"` // buat rotate turn direction icon di front-end.
	TurnType    string     `json:"turn_type"`
}

func NewDrivingDirection(ins Instruction, description string, prevETA, prevDist float64,
	edgeIDs []int32, polyline string, turnBearing float64) DrivingDirection {
	return DrivingDirection{
		Instruction: description,
		Point:       ins.Point,
		StreetName:  ins.Name,
		ETA:         util.RoundFloat(prevETA, 2),
		Distance:    util.RoundFloat(prevDist, 2),
		EdgeIDs:     edgeIDs,
		Polyline:    polyline,
		TurnBearing: util.RoundFloat(turnBearing, 2),
		TurnType:    ins.TurnType,
	}
}
