#!/usr/bin/env bash

dot -T pdf euslisp-relation.dot -o euslisp-relation.pdf
evince euslisp-relation.pdf
