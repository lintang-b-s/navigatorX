package server

import (
	"errors"
	"fmt"
)

type Error struct {
	orig error
	msg  string
	code error
}

func (e *Error) Error() string {
	if e.orig != nil {
		return fmt.Sprintf("%s", e.msg)
	}

	return e.msg
}

func (e *Error) Unwrap() error {
	return e.orig
}

func WrapErrorf(orig error, code error, format string, a ...interface{}) error {
	return &Error{
		code: code,
		orig: orig,
		msg:  fmt.Sprintf(format, a...),
	}
}

func (e *Error) Code() error {
	return e.code
}

var (
	ErrInternalServerError = errors.New("internal Server Error")
	ErrNotFound = errors.New("your requested Item is not found")
	ErrConflict = errors.New("your Item already exist")
	ErrBadParamInput = errors.New("given Param is not valid")
)

var MessageInternalServerError string = "internal server error"
