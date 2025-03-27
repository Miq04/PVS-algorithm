#!/bin/bash

data_folder="D:/randomtrips-test"
sumo_path="E:/Sumo"
my_folder=$(pwd)
time=$(date +"%Y%m%d%H%M%S")

mkdir -p "${my_folder}/data/${time}"
cd "${data_folder}"
python randomTrips.py -n map.net.xml --trip-attributes="type=\"myType\"" --additional-file type.add.xml --edge-permission passenger --fringe-factor 100 --validate --random
cp ./map.net.xml ./routes.rou.xml ./map.sumocfg "${sumo_path}/tools/"

cd "${sumo_path}/tools/"
sumo -c map.sumocfg --fcd-output sumoTrace.xml
mkdir -p xml
cd xml
python xml2csv.py ../sumoTrace.xml --output sumoTrace.csv
cp sumoTrace.csv "${my_folder}/data/${time}"

cd "${my_folder}/create_data"
python generate_flow_dataset.py --input-path "${my_folder}/data/${time}"
cd "${my_folder}"