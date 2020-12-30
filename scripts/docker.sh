#!/bin/bash

rootDir=$(dirname $0)
rootDir=$(cd ${rootDir}/.. && pwd)
targetDir=$(cd ${rootDir} && basename $(pwd))

docker run -it --mount src="${rootDir}",target=/${targetDir},type=bind ${targetDir}:latest /bin/bash
