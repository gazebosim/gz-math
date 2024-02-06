#!/bin/bash

#Movements in these cases are mostly smooth, and any abrupt movements caused by these cases are present in Slerp as well as Squad interpolation
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x: 27.2, y:-1.7215,z:2.2255}, orientation:{x:-0.1466788, y:0.0344671, z:0.9623708, w:0.2261413}}'
sleep 2
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x:24.081968307495117,y:1.0910710096359253,z:1.194319486618042},orientation:{x:0.10836052149534225,y:-0.0045970650389790535,z:-0.99320769309997559,w:-0.042135711759328842}}'
sleep 2
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x:22.081968307495117,y:1.0910710096359253,z:1.194319486618042},orientation:{x:-1.10836052149534225,y:1.0045970650389790535,z:1.99320769309997559,w:1.042135711759328842}}'
sleep 2
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x: 27.2, y:-1.7215,z:2.2255}, orientation:{x:-0.1466788, y:0.0344671, z:0.9623708, w:0.2261413}}'
sleep 2
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x: 31.2, y:-6.7215,z:7.2255}, orientation:{x:-0.1466788, y:0.0344671, z:0.9623708, w:0.2261413}}'
sleep 2
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x: 27.2, y:-1.7215,z:2.2255}, orientation:{x:-23.1466788, y:-23.0344671, z:23.9623708, w:3.2261413}}'
sleep 2
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x: 27.2, y:-1.7215,z:2.2255}, orientation:{x:-21.1466788, y:-21.0344671, z:21.9623708, w:1.2261413}}'
sleep 2
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x:0.5,y:1.0,z:1.0},orientation:{x:0.15,y:-0.05,z:-1.0,w:-0.05}}'
sleep 2
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 500 --req 'pose: {position:{x:1.5,y:1.0,z:1.0},orientation:{x:0.15,y:-0.05,z:-1.0,w:-1.05}}'
