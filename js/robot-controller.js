class RobotController {
    constructor(robotCar, pathfinding, rooms) {
        this.robotCar = robotCar;
        this.pathfinding = pathfinding;
        this.rooms = rooms;
        this.aiTargets = [];
        this.currentTargetIndex = 0;
        this.isAIMode = true;
        this.keys = {};
        
        // WebSocket manager reference for status updates
        this.websocketManager = null;
        
        // Movement settings
        this.robotSpeed = 7.5;           // units/sec (feels snappy but stable)
        this.robotRotationSpeed = 4.0;   // rad/sec
        
        // Path following settings
        this.waypointReachDistance = 0.45;  // must get closer before "reached"
        this.minWaypointDistance = 0.9;    // keep corners; we no longer form diagonals
        
        // Debug logging control
        this.lastLogTime = 0;
        this.logInterval = 500;
        
        // Obstacle collision counter to prevent infinite loops
        this.obstacleCollisionCount = 0;
        this.maxObstacleCollisions = 12;
        
        // Final destination tracking
        this.finalDestination = null;
        this.currentRoomKey = null;  // Track which room we're navigating to
    }
    
    // Set AI mode
    setAIMode(isAI) {
        this.isAIMode = isAI;
        if (!isAI) {
            this.aiTargets = [];
            this.currentTargetIndex = 0;
        } else {
            // When switching to AI mode, snap to nearest walkable cell center
            const snap = this.pathfinding.findNearestWalkablePosition(this.robotCar.position, 5);
            if (snap) {
                console.log(`Snapping robot from (${this.robotCar.position.x.toFixed(1)}, ${this.robotCar.position.z.toFixed(1)}) to clean position (${snap.x.toFixed(1)}, ${snap.z.toFixed(1)})`);
                this.robotCar.position.copy(snap);
            }
        }
    }
    
    // Set WebSocket manager reference
    setWebSocketManager(websocketManager) {
        this.websocketManager = websocketManager;
    }
    
    // Handle manual positioning - robot can be placed anywhere manually
    handleManualPositioning() {
        // Reset any existing AI targets when switching to manual mode
        if (!this.isAIMode) {
            this.aiTargets = [];
            this.currentTargetIndex = 0;
            this.obstacleCollisionCount = 0;
        }
    }
    
    // Check if current position is valid for pathfinding
    isCurrentPositionValid() {
        return this.pathfinding.isValidPosition(this.robotCar.position);
    }
    
    // Go to a specific room
    goToRoom(roomKey, roomPosition) {
        if (!this.isAIMode) return;
        
        console.log(`=== GOING TO ${roomKey} ===`);
        console.log(`Robot at: (${this.robotCar.position.x.toFixed(1)}, ${this.robotCar.position.z.toFixed(1)})`);
        console.log(`Target: (${roomPosition.x.toFixed(1)}, ${roomPosition.z.toFixed(1)})`);
        
        // Validate current position first
        this.validateCurrentPosition();
        
        // Get the predefined entrance for this room
        const room = this.rooms[roomKey];
        if (!room || !room.entrance) {
            console.log(`No entrance defined for room ${roomKey}`);
            return `AI: Error - No entrance defined for ${roomKey}`;
        }
        
        const entrancePosition = room.entrance;
        console.log(`Room entrance: (${entrancePosition.x.toFixed(1)}, ${entrancePosition.z.toFixed(1)})`);
        
        // Check if robot is already at the entrance
        const distanceToEntrance = this.robotCar.position.distanceTo(entrancePosition);
        if (distanceToEntrance < this.waypointReachDistance) {
            console.log(`Robot already at ${roomKey} entrance`);
            this.notifyDestinationReached(roomKey, entrancePosition);
            return `AI: Already at ${roomKey} entrance`;
        }
        
        // Find path using pathfinding to the entrance
        const path = this.pathfinding.findPath(this.robotCar.position, entrancePosition);
        
        if (path && path.length > 0) {
            // Debug: Visualize the generated path
            this.pathfinding.debugPathOnGrid(path);
            
            // Validate that the path doesn't go through obstacles
            if (!this.pathfinding.validatePath(path)) {
                console.log('Path validation failed - path goes through obstacles');
                return `AI: Error - No path found to ${roomKey} (path blocked by obstacles)`;
            }
            
            // Filter waypoints to be appropriately spaced
            this.aiTargets = this.filterWaypoints(path);
            this.currentTargetIndex = 0;
            
            // Store the final destination for completion checking
            this.finalDestination = entrancePosition.clone();
            this.currentRoomKey = roomKey;  // Track which room we're navigating to
            
            // Reset obstacle collision counter for new path
            this.obstacleCollisionCount = 0;
            
            console.log(`Path found: ${path.length} waypoints, filtered to ${this.aiTargets.length} waypoints`);
            console.log(`Final destination: (${this.finalDestination.x.toFixed(1)}, ${this.finalDestination.z.toFixed(1)})`);
            this.aiTargets.forEach((wp, i) => {
                console.log(`WP${i}: (${wp.x.toFixed(1)}, ${wp.z.toFixed(1)})`);
            });
            
            return `AI: Found valid path to ${roomKey} entrance with ${this.aiTargets.length} waypoints`;
        } else {
            console.log(`No path found to ${roomKey} entrance, attempting fallback strategy`);
            
            // Fallback: Try to find a path to a nearby walkable position first
            const walkablePos = this.pathfinding.findNearestWalkablePosition(this.robotCar.position);
            if (walkablePos) {
                console.log(`Found walkable position: (${walkablePos.x.toFixed(1)}, ${walkablePos.z.toFixed(1)})`);
                
                // Try pathfinding from the walkable position
                const fallbackPath = this.pathfinding.findPath(walkablePos, entrancePosition);
                if (fallbackPath && fallbackPath.length > 0) {
                    console.log(`Fallback path found with ${fallbackPath.length} waypoints`);
                    
                    // Add walkable position as first waypoint, then the rest
                    this.aiTargets = [walkablePos.clone()];
                    const filteredWaypoints = this.filterWaypoints(fallbackPath);
                    this.aiTargets.push(...filteredWaypoints);
                    this.currentTargetIndex = 0;
                    this.finalDestination = entrancePosition.clone();
                    this.obstacleCollisionCount = 0;
                    
                    console.log(`Fallback path: ${this.aiTargets.length} total waypoints`);
                    return `AI: Found fallback path to ${roomKey} entrance with ${this.aiTargets.length} waypoints`;
                }
            }
            
            console.log(`No path found to ${roomKey} entrance`);
            return `AI: Error - No path found to ${roomKey} entrance`;
        }
    }
    
    // Notify WebSocket manager when destination is reached
    notifyDestinationReached(roomKey, position) {
        if (this.websocketManager) {
            this.websocketManager.sendMessage({
                type: 'robot_arrival',
                data: {
                    destination: roomKey,
                    position: position,
                    timestamp: new Date().toISOString()
                }
            });
        }
    }
    
    // Filter waypoints to appropriate spacing
    filterWaypoints(path) {
        if (!path || path.length <= 2) return path;
        
        const toGrid = (v) => this.pathfinding.worldToGrid(v);
        const g = path.map(toGrid);
        
        const kept = [path[0]];
        let prevDir = null;
        
        for (let i = 1; i < g.length; i++) {
            const dx = Math.sign(g[i].x - g[i - 1].x);
            const dz = Math.sign(g[i].z - g[i - 1].z);
            const dir = `${dx},${dz}`; // one of "1,0","-1,0","0,1","0,-1"
            
            if (prevDir === null) {
                prevDir = dir;
                continue;
            }
            
            if (dir !== prevDir) {
                // direction changed at i -> keep the LAST node of the previous run (i-1)
                kept.push(path[i - 1]);
                prevDir = dir;
            }
        }
        
        // keep the final destination
        const last = path[path.length - 1];
        if (kept[kept.length - 1] !== last) kept.push(last);
        
        // light spacing guard (won't remove corners)
        const spaced = [kept[0]];
        for (let i = 1; i < kept.length; i++) {
            if (kept[i].distanceTo(spaced[spaced.length - 1]) >= this.minWaypointDistance) {
                spaced.push(kept[i]);
            }
        }
        
        console.log(`Filtered ${path.length} waypoints down to ${spaced.length} (axis-aligned corners kept)`);
        return spaced;
    }
    
    // Go to sequence of rooms
    goToRoomSequence(roomKeys, rooms) {
        if (!this.isAIMode) return;
        
        this.aiTargets = [];
        let currentPos = this.robotCar.position.clone();
        
        roomKeys.forEach(roomKey => {
            const room = rooms[roomKey];
            if (room && room.entrance) {
                const path = this.pathfinding.findPath(currentPos, room.entrance);
                if (path && path.length > 0 && this.pathfinding.validatePath(path)) {
                    // Add filtered waypoints
                    const filteredWaypoints = this.filterWaypoints(path);
                    // Skip first waypoint if it's the same as current position
                    for (let i = 1; i < filteredWaypoints.length; i++) {
                        this.aiTargets.push(filteredWaypoints[i]);
                    }
                    currentPos = room.entrance.clone();
                } else {
                    // Try to salvage with nearest walkable near the entrance
                    const near = this.pathfinding.findNearestWalkablePosition(room.entrance) || null;
                    if (near) {
                        const path2 = this.pathfinding.findPath(currentPos, near);
                        if (path2 && this.pathfinding.validatePath(path2)) {
                            this.aiTargets.push(...this.filterWaypoints(path2).slice(1));
                            currentPos = near.clone();
                        }
                    }
                }
            }
        });
        
        this.currentTargetIndex = 0;
        
        // Reset obstacle collision counter for new sequence
        this.obstacleCollisionCount = 0;
        
        // Set current room key for the first destination
        if (roomKeys.length > 0) {
            this.currentRoomKey = roomKeys[0];
        }
        
        return `AI: Starting route with ${this.aiTargets.length} waypoints`;
    }
    
    // Stop AI movement
    stopAI() {
        this.aiTargets = [];
        this.currentTargetIndex = 0;
        this.finalDestination = null;
        this.currentRoomKey = null;  // Clear current room tracking
        return 'AI: Stopped';
    }
    
    // Update robot movement
    updateRobotMovement(dt = 1/60) {
        if (this.isAIMode && this.aiTargets.length > 0 && this.currentTargetIndex < this.aiTargets.length) {
            this.updateAIMovement(dt);
        } else if (!this.isAIMode) {
            this.updateManualMovement(dt);
        }
        
        // Keep robot on ground
        this.robotCar.position.y = 0;
        
        // Check if robot is stuck
        // this.checkIfStuck(); // Removed stuck detection
    }
    
    // Check if robot is stuck
    // checkIfStuck() { // Removed stuck detection
    //     const currentPos = this.robotCar.position;
    //     const distanceMoved = currentPos.distanceTo(this.lastPosition);
        
    //     if (distanceMoved < 0.1) {
    //         this.stuckTimer += 16; // Assuming 60fps
    //         if (this.stuckTimer > this.maxStuckTime) {
    //             console.log('Robot appears to be stuck, attempting recovery...');
    //             this.attemptStuckRecovery();
    //             this.stuckTimer = 0;
    //         }
    //     } else {
    //         this.stuckTimer = 0;
    //     }
        
    //     this.lastPosition = currentPos.clone();
    // }
    
    // Attempt to recover from being stuck
    // attemptStuckRecovery() { // Removed stuck detection
    //     if (this.aiTargets.length > 0) {
    //         // Try to find a new path to the current target
    //         const currentTarget = this.aiTargets[this.currentTargetIndex];
    //         console.log('Attempting to find new path to current target:', currentTarget);
            
    //         const newPath = this.pathfinding.findPath(this.robotCar.position, currentTarget);
    //         if (newPath && newPath.length > 1) {
    //             // Replace current path with new one
    //             this.aiTargets = this.filterWaypoints(newPath);
    //             this.currentTargetIndex = 0;
    //             console.log('New path found for stuck recovery');
    //         } else {
    //             // Skip this target and move to next
    //             console.log('Could not find new path, skipping target');
    //             this.currentTargetIndex++;
    //             if (this.currentTargetIndex >= this.aiTargets.length) {
    //                 this.stopAI();
    //             }
    //         }
    //     }
    // }
    
    // AI movement logic
    updateAIMovement(dt) {
        const currentTarget = this.aiTargets[this.currentTargetIndex];
        const direction = currentTarget.clone().sub(this.robotCar.position);
        const distance = direction.length();
        
        // Controlled logging - only log occasionally
        const now = Date.now();
        if (now - this.lastLogTime > this.logInterval) {
            console.log(`Moving to waypoint ${this.currentTargetIndex + 1}/${this.aiTargets.length}: (${currentTarget.x.toFixed(1)}, ${currentTarget.z.toFixed(1)})`);
            console.log(`Distance: ${distance.toFixed(1)}`);
            const currG = this.pathfinding.worldToGrid(this.robotCar.position);
            const targG = this.pathfinding.worldToGrid(currentTarget);
            const sameCell = (currG.x === targG.x && currG.z === targG.z);
            const veryClose = distance <= 0.5;
            console.log(`Grid: Robot(${currG.x},${currG.z}) Target(${targG.x},${targG.z}) SameCell:${sameCell} VeryClose:${veryClose}`);
            this.lastLogTime = now;
        }
        
        // Check if we're in the same grid cell as the waypoint (prevents hovering)
        const currG = this.pathfinding.worldToGrid(this.robotCar.position);
        const targG = this.pathfinding.worldToGrid(currentTarget);
        const sameCell = (currG.x === targG.x && currG.z === targG.z);
        
        if (distance > this.waypointReachDistance || !sameCell) {
            direction.normalize();
            
            // Direction to current waypoint
            const toTarget = currentTarget.clone().sub(this.robotCar.position);
            const absX = Math.abs(toTarget.x);
            const absZ = Math.abs(toTarget.z);
            
            // If we're in the same grid row or column as the waypoint, walk purely along that axis.
            const currG = this.pathfinding.worldToGrid(this.robotCar.position);
            const targG = this.pathfinding.worldToGrid(currentTarget);
            if (currG.x === targG.x && currG.z !== targG.z) {
                // same column: move only in Z
                direction.set(0, 0, Math.sign(toTarget.z));
            } else if (currG.z === targG.z && currG.x !== targG.x) {
                // same row: move only in X
                direction.set(Math.sign(toTarget.x), 0, 0);
            } else {
                // otherwise (just after a turn), use the regular vector
                direction.copy(toTarget).normalize();
            }
            
            // snap tolerance ~ 0.2 * gridSize (adjust if your grid differs)
            const snapTol = 0.35;
            
            // If we're essentially moving along Z, lock X to the current cell center
            if (absZ > absX) {
                const gx = this.pathfinding.worldToGrid(this.robotCar.position).x;
                const cx = this.pathfinding.gridToWorld({x: gx, z: 0}).x; // center of current X cell
                if (Math.abs(this.robotCar.position.x - cx) <= snapTol) {
                    this.robotCar.position.x = cx;
                }
            } else if (absX > absZ) {
                // moving along X, lock Z to current cell center
                const gz = this.pathfinding.worldToGrid(this.robotCar.position).z;
                const cz = this.pathfinding.gridToWorld({x: 0, z: gz}).z;
                if (Math.abs(this.robotCar.position.z - cz) <= snapTol) {
                    this.robotCar.position.z = cz;
                }
            }
            
            // Nudge away from obstacles only when not close to the waypoint
            const nearCutoff = 1.5 * this.pathfinding.gridSize;
            if (distance > nearCutoff) {
                const repel = this.pathfinding.shortRangeRepulsion(this.robotCar.position);
                // scale repulsion so it never overpowers the main direction
                const scale = Math.min(distance / (4 * this.pathfinding.gridSize), 0.25);
                repel.multiplyScalar(scale);
                direction.add(repel).normalize();
            }
            
            // Calculate next position with step clamping
            const step = Math.min(this.robotSpeed * dt, 0.8 * this.pathfinding.gridSize); // clamp to 0.8×gridSize
            const nextPosition = this.robotCar.position.clone().add(direction.clone().multiplyScalar(step));
            
            // Check if next position is valid
            if (this.pathfinding.isValidPosition(nextPosition)) {
                this.robotCar.position.copy(nextPosition);
            } else {
                // Try shrinking the step to stay inside the current walkable cell
                let tried = 0, ok = false;
                let tentative = nextPosition.clone();
                while (tried < 3 && !ok) {
                    tentative.copy(this.robotCar.position).add(direction.clone().multiplyScalar(step * Math.pow(0.5, tried + 1)));
                    ok = this.pathfinding.isValidPosition(tentative);
                    tried++;
                }
                if (ok) {
                    this.robotCar.position.copy(tentative);
                    return; // continue next frame
                }
                
                // As a second option, try axis-aligned nudge (prefer the axis with smaller angle)
                const axisFirst = Math.abs(direction.x) > Math.abs(direction.z) ? 'x' : 'z';
                const axisVec = new THREE.Vector3(axisFirst === 'x' ? Math.sign(direction.x) : 0, 0,
                                               axisFirst === 'z' ? Math.sign(direction.z) : 0);
                tentative.copy(this.robotCar.position).add(axisVec.multiplyScalar(step * 0.8));
                if (this.pathfinding.isValidPosition(tentative)) {
                    this.robotCar.position.copy(tentative);
                    return;
                }
                
                // If both fail, then replan
                console.log('Hit obstacle, attempting to find new path...');
                console.log(`Robot at: (${this.robotCar.position.x.toFixed(1)}, ${this.robotCar.position.z.toFixed(1)})`);
                console.log(`Trying to reach: (${currentTarget.x.toFixed(1)}, ${currentTarget.z.toFixed(1)})`);
                console.log(`Next position would be: (${nextPosition.x.toFixed(1)}, ${nextPosition.z.toFixed(1)})`);
                
                // Debug the grid around the robot and target
                this.pathfinding.debugGridPosition(this.robotCar.position);
                this.pathfinding.debugGridPosition(currentTarget);
                this.pathfinding.debugGridPosition(nextPosition);
                this.pathfinding.debugGridAround(this.robotCar.position, 3);
                
                this.obstacleCollisionCount++;
                
                if (this.obstacleCollisionCount > this.maxObstacleCollisions) {
                    console.log('Too many obstacle collisions, attempting recovery...');
                    
                    // Try to find a nearby walkable position
                    const walkablePos = this.pathfinding.findNearestWalkablePosition(this.robotCar.position);
                    if (walkablePos) {
                        console.log(`Moving robot to nearby walkable position: (${walkablePos.x.toFixed(1)}, ${walkablePos.z.toFixed(1)})`);
                        this.robotCar.position.copy(walkablePos);
                        
                        // Reset collision counter and try to find new path
                        this.obstacleCollisionCount = 0;
                        
                        if (this.finalDestination) {
                            const recoveryPath = this.pathfinding.findPath(this.robotCar.position, this.finalDestination);
                            if (recoveryPath && this.pathfinding.validatePath(recoveryPath)) {
                                this.aiTargets = this.filterWaypoints(recoveryPath);
                                this.currentTargetIndex = 0;
                                console.log(`Recovery path found with ${this.aiTargets.length} waypoints`);
                                return;
                            }
                        }
                    }
                    
                    console.log('Recovery failed, trying to skip current target');
                    this.currentTargetIndex++;
                    if (this.currentTargetIndex >= this.aiTargets.length) { this.stopAI(); }
                    return;
                }
                
                // Try to find a new path from current position to current target
                const newPath = this.pathfinding.findPath(this.robotCar.position, currentTarget);
                
                if (newPath && this.pathfinding.validatePath(newPath)) {
                    // Replace current path with new one
                    this.aiTargets = this.filterWaypoints(newPath);
                    this.currentTargetIndex = 0;
                    console.log(`New path found with ${this.aiTargets.length} waypoints`);
                } else {
                    // Cannot find valid path, try to find path to final destination instead
                    if (this.finalDestination) {
                        console.log('Cannot find path to current waypoint, trying direct path to final destination');
                        const directPath = this.pathfinding.findPath(this.robotCar.position, this.finalDestination);
                        
                        if (directPath && this.pathfinding.validatePath(directPath)) {
                            this.aiTargets = this.filterWaypoints(directPath);
                            this.currentTargetIndex = 0;
                            console.log(`Direct path to final destination found with ${this.aiTargets.length} waypoints`);
                        } else {
                            // Still can't find path, try to skip this target
                            console.log('Cannot find any valid path, skipping current target');
                            this.currentTargetIndex++;
                            
                            if (this.currentTargetIndex >= this.aiTargets.length) {
                                console.log('No more targets, stopping AI');
                                this.stopAI();
                                return;
                            }
                        }
                    } else {
                        // Cannot find valid path, try to skip this target
                        console.log('Cannot find valid path, skipping current target');
                        this.currentTargetIndex++;
                        
                        if (this.currentTargetIndex >= this.aiTargets.length) {
                            console.log('No more targets, stopping AI');
                            this.stopAI();
                            return;
                        }
                    }
                }
                
                return; // Skip movement this frame
            }
            
            // Rotate toward target
            const targetAngle = Math.atan2(direction.x, direction.z);
            const currentAngle = this.robotCar.rotation.y;
            let angleDiff = targetAngle - currentAngle;
            
            // Normalize angle difference
            if (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            if (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
            
            this.robotCar.rotation.y += angleDiff * this.robotRotationSpeed * dt;
        } else {
            // Reached current target (distance small AND in same grid cell)
            console.log(`Reached waypoint ${this.currentTargetIndex + 1}/${this.aiTargets.length}`);
            // Snap to the exact waypoint center to avoid off-by-one cell drift
            this.robotCar.position.copy(currentTarget);
            this.obstacleCollisionCount = 0;
            this.currentTargetIndex++;
            
            if (this.currentTargetIndex >= this.aiTargets.length) {
                // Check if we've actually reached the final destination
                if (this.finalDestination) {
                    const distanceToFinal = this.robotCar.position.distanceTo(this.finalDestination);
                    if (distanceToFinal <= this.waypointReachDistance) {
                        console.log(`Route completed! Reached ${this.finalDestination.x.toFixed(1)}, ${this.finalDestination.z.toFixed(1)}`);
                        
                        // Notify WebSocket manager of completion
                        if (this.websocketManager) {
                            this.websocketManager.sendMessage({
                                type: 'robot_arrival',
                                data: {
                                    destination: this.currentRoomKey,
                                    position: this.robotCar.position,
                                    timestamp: new Date().toISOString()
                                }
                            });
                        }
                        
                        this.aiTargets = [];
                        this.currentTargetIndex = 0;
                        this.finalDestination = null;
                        this.currentRoomKey = null;  // Clear current room tracking
                    } else {
                        // We reached the last waypoint but not the final destination
                        // Add the final destination as the next target
                        console.log(`Reached last waypoint but not final destination. Distance: ${distanceToFinal.toFixed(1)}`);
                        console.log(`Adding final destination as next target: (${this.finalDestination.x.toFixed(1)}, ${this.finalDestination.z.toFixed(1)})`);
                        this.aiTargets.push(this.finalDestination.clone());
                    }
                } else {
                    // No final destination stored, assume we're done
                    console.log('Route completed! (No final destination stored)');
                    this.aiTargets = [];
                    this.currentTargetIndex = 0;
                    this.currentRoomKey = null;  // Clear current room tracking
                }
            }
        }
    }
    
    // Manual movement logic
    updateManualMovement(dt) {
        let newPosition = this.robotCar.position.clone();
        
        if (this.keys['w']) {
            newPosition.x += Math.sin(this.robotCar.rotation.y) * this.robotSpeed * dt;
            newPosition.z += Math.cos(this.robotCar.rotation.y) * this.robotSpeed * dt;
        }
        if (this.keys['s']) {
            newPosition.x -= Math.sin(this.robotCar.rotation.y) * this.robotSpeed * dt;
            newPosition.z -= Math.cos(this.robotCar.rotation.y) * this.robotSpeed * dt;
        }
        if (this.keys['a']) {
            this.robotCar.rotation.y += this.robotRotationSpeed * dt;
        }
        if (this.keys['d']) {
            this.robotCar.rotation.y -= this.robotRotationSpeed * dt;
        }
        
        // Only move if new position is valid
        if (this.pathfinding.isValidPosition(newPosition)) {
            this.robotCar.position.copy(newPosition);
        } else {
            // Provide feedback that position is invalid
            console.log('Manual movement blocked by obstacle');
        }
        
        // Handle manual positioning changes
        this.handleManualPositioning();
    }
    
    // Handle keyboard input
    handleKeyDown(key) {
        this.keys[key.toLowerCase()] = true;
    }
    
    handleKeyUp(key) {
        this.keys[key.toLowerCase()] = false;
    }
    
    // Get current status
    getStatus() {
        if (this.aiTargets.length > 0) {
            return `Targets: ${this.currentTargetIndex + 1}/${this.aiTargets.length}`;
        }
        return this.isAIMode ? 'Following AI' : 'Manual Control';
    }
    
    // Validate current position and provide feedback
    validateCurrentPosition() {
        const isValid = this.isCurrentPositionValid();
        const gridPos = this.pathfinding.worldToGrid(this.robotCar.position);
        
        console.log(`=== POSITION VALIDATION ===`);
        console.log(`Robot at: (${this.robotCar.position.x.toFixed(1)}, ${this.robotCar.position.z.toFixed(1)})`);
        console.log(`Grid position: (${gridPos.x}, ${gridPos.z})`);
        console.log(`Position valid: ${isValid ? 'YES' : 'NO'}`);
        
        if (!isValid) {
            console.log('WARNING: Robot is in an invalid position!');
            console.log('This may cause pathfinding to fail.');
        }
        
        console.log(`=== END VALIDATION ===`);
        return isValid;
    }
}
