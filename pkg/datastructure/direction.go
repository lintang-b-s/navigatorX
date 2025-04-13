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
	Point        Coordinate
	RawName      bool
	Sign         int
	Name         string
	Distance     float64
	Time         float64
	ExtraInfo    map[string]interface{}
	IsRoundabout bool
	Roundabout   RoundaboutInstruction
}

func NewInstruction(sign int, name string, p Coordinate, isRoundAbout bool) Instruction {
	var roundabout RoundaboutInstruction
	var ins Instruction
	if isRoundAbout {
		roundabout = NewRoundaboutInstruction()
		ins = Instruction{
			Sign:         sign,
			Name:         name,
			Point:        p,
			ExtraInfo:    make(map[string]interface{}, 3),
			Roundabout:   roundabout,
			Time:         0,
			IsRoundabout: true,
			Distance:     0,
		}
	} else {
		ins = Instruction{
			Sign:         sign,
			Name:         name,
			Point:        p,
			ExtraInfo:    make(map[string]interface{}, 3),
			IsRoundabout: false,
			Time:         0,
			Distance:     0,
		}
		return ins
	}

	return ins
}

func NewInstructionWithRoundabout(sign int, name string, p Coordinate, isRoundAbout bool, roundabout RoundaboutInstruction) Instruction {
	ins := Instruction{
		Sign:         sign,
		Name:         name,
		Point:        p,
		ExtraInfo:    make(map[string]interface{}, 3),
		Roundabout:   roundabout,
		IsRoundabout: isRoundAbout,
		Time:         0,
		Distance:     0,
	}
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
		dir := getDirectionDescription(sign, *instr)
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
func getDirectionDescription(sign int, instruction Instruction) string {
	switch sign {
	case U_TURN_UNKNOWN:
		return "Make U-turn"
	case U_TURN_RIGHT:
		return "Make U-turn right"
	case U_TURN_LEFT:
		return "Make U-turn left"
	case KEEP_LEFT:
		return "Keep left"
	case TURN_SHARP_LEFT:
		return "Turn sharp left"
	case TURN_LEFT:
		return "Turn left"
	case TURN_SLIGHT_LEFT:
		return "Turn slight left"
	case TURN_SLIGHT_RIGHT:
		return "Turn slight right"
	case TURN_RIGHT:
		return "Turn right"
	case TURN_SHARP_RIGHT:
		return "Turn sharp right"
	case KEEP_RIGHT:
		return "Keep right"
	case USE_ROUNDABOUT:
		if !instruction.Roundabout.Exited {
			return "Enter the roundabout"
		}
		roundaboutDir := "clockwise" // bundaran  di indo selalu clockwise

		return fmt.Sprintf("At Roundabout, take the exit point %d %s", instruction.Roundabout.ExitNumber, roundaboutDir)

	default:
		return ""
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
	Instruction string
	Point       Coordinate
	StreetName  string
	ETA         float64
	Distance    float64
}

func NewDrivingDirection(ins Instruction, description string, prevETA, prevDist float64) DrivingDirection {
	return DrivingDirection{
		Instruction: description,
		Point:       ins.Point,
		StreetName:  ins.Name,
		ETA:         util.RoundFloat(prevETA, 2),
		Distance:    util.RoundFloat(prevDist, 2),
	}
}
