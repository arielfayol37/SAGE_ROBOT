class RobotCarSimulation {
    constructor() {
        console.log('RobotCarSimulation constructor called');
        
        // Enhanced console logging
        this.log = (message, type = 'info') => {
            const timestamp = new Date().toLocaleTimeString();
            const prefix = `[${timestamp}] [${type.toUpperCase()}]`;
            console.log(`${prefix} ${message}`);
        };
        
        try {
            // Test if Three.js is actually working
            if (typeof THREE === 'undefined') {
                throw new Error('THREE is undefined!');
            }
            
            console.log('THREE object:', THREE);
            console.log('THREE.Scene:', THREE.Scene);
            
            this.scene = null;
            this.camera = null;
            this.renderer = null;
            this.robotCar = null;
            this.websocket = null;
            
            // Initialize modules
            this.pathfinding = null;
            this.robotController = null;
            this.buildingGenerator = null;
            this.rooms = {};
            
            // WebSocket manager for AI backend communication
            this.websocketManager = null;
            
            console.log('About to call init()');
            this.init();
            console.log('About to call setupEventListeners()');
            this.setupEventListeners();
            console.log('About to call animate()');
            this.animate();
            
            console.log('RobotCarSimulation initialized successfully');
        } catch (error) {
            console.error('Error in RobotCarSimulation constructor:', error);
            alert('Error creating simulation: ' + error.message);
        }
    }
    
    init() {
        try {
            console.log('Starting init()...');
            
            // Create scene
            console.log('Creating scene...');
            this.scene = new THREE.Scene();
            this.scene.background = new THREE.Color(0x87CEEB); // Sky blue
            console.log('Scene created:', this.scene);
            
            // Create camera
            console.log('Creating camera...');
            this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
            this.camera.position.set(0, 30, 40);
            this.camera.lookAt(0, 0, 0);
            
            // Camera modes: 'stabilized_follow' | 'chase' | 'first_person'
            this.cameraMode = 'stabilized_follow';
            this._smoothCamPos = this.camera.position.clone();
            this._smoothCamLook = new THREE.Vector3();
            this._smoothCamQuat = new THREE.Quaternion();
            
            console.log('Camera created:', this.camera);
            
            // Create renderer
            console.log('Creating renderer...');
            this.renderer = new THREE.WebGLRenderer({ antialias: true });
            // Better output & lighting model
            if ('outputColorSpace' in this.renderer) {
                this.renderer.outputColorSpace = THREE.SRGBColorSpace;
            } else {
                this.renderer.outputEncoding = THREE.sRGBEncoding;
            }
            if (THREE.ACESFilmicToneMapping !== undefined) {
                this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
            }
            this.renderer.toneMappingExposure = 1.0;
            this.renderer.physicallyCorrectLights = true;
            // Soft shadows
            this.renderer.shadowMap.enabled = true;
            this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
            // Reduce shimmer on HiDPI screens
            this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
            this.renderer.setSize(window.innerWidth, window.innerHeight);
            
            const container = document.getElementById('canvas-container');
            console.log('Container found:', container);
            container.appendChild(this.renderer.domElement);
            console.log('Renderer added to DOM');
            
            // Add lighting
            console.log('Setting up lighting...');
            this.setupLighting();
            
            // Initialize pathfinding
            console.log('Initializing pathfinding...');
            this.pathfinding = new Pathfinding();
            this.pathfinding.createNavigationGrid();
            
            // Create building
            console.log('Creating building...');
            this.buildingGenerator = new BuildingGenerator(this.scene, this.pathfinding);
            this.buildingGenerator.createBuilding();
            this.rooms = this.buildingGenerator.getRooms();
            
            // Test grid after building creation
            console.log('=== TESTING GRID AFTER BUILDING CREATION ===');
            const testPositions = [
                new THREE.Vector3(0, 0, 0),      // Center (should be walkable)
                new THREE.Vector3(-80, 0, -80),  // ER room center (should be obstacle)
                new THREE.Vector3(-80, 0, -67.5), // ER entrance (should be walkable)
                new THREE.Vector3(-30, 0, -30),  // Where robot got stuck (should be walkable)
                new THREE.Vector3(-32, 0, -31)   // Target where robot got stuck (should be walkable)
            ];
            
            testPositions.forEach((pos, i) => {
                const isValid = this.pathfinding.isValidPosition(pos);
                console.log(`Test ${i}: (${pos.x.toFixed(1)}, ${pos.z.toFixed(1)}) -> ${isValid ? 'WALKABLE' : 'OBSTACLE'}`);
            });
            
            // Test pathfinding with a simple case
            console.log('=== TESTING PATHFINDING ===');
            const testStart = new THREE.Vector3(0, 0, 0);
            const testEnd = new THREE.Vector3(10, 0, 0);
            const testPath = this.pathfinding.findPath(testStart, testEnd);
            if (testPath) {
                console.log(`Pathfinding test: Found path with ${testPath.length} waypoints`);
                this.pathfinding.debugPathOnGrid(testPath, 5);
            } else {
                console.log('Pathfinding test: No path found');
            }
            
            // Test pathfinding from different starting positions
            const testPositions2 = [
                new THREE.Vector3(0, 0, 0),      // Center
                new THREE.Vector3(-50, 0, 0),    // Left side
                new THREE.Vector3(50, 0, 0),     // Right side
                new THREE.Vector3(0, 0, -50),    // Back
                new THREE.Vector3(0, 0, 50)      // Front
            ];
            
            testPositions2.forEach((startPos, i) => {
                if (this.pathfinding.isValidPosition(startPos)) {
                    const testPath2 = this.pathfinding.findPath(startPos, testEnd);
                    if (testPath2) {
                        console.log(`Test ${i}: Path from (${startPos.x}, ${startPos.z}) -> (${testEnd.x}, ${testEnd.z}): ${testPath2.length} waypoints`);
                    } else {
                        console.log(`Test ${i}: No path from (${startPos.x}, ${startPos.z}) -> (${testEnd.x}, ${testEnd.z})`);
                    }
                } else {
                    console.log(`Test ${i}: Starting position (${startPos.x}, ${startPos.z}) is invalid`);
                }
            });
            
            console.log('=== END PATHFINDING TEST ===');
            console.log('=== END GRID TEST ===');
            
            // Create robot car
            console.log('Creating robot car...');
            this.createRobotCar();
            
            // Initialize robot controller
            console.log('Initializing robot controller...');
            this.robotController = new RobotController(this.robotCar, this.pathfinding, this.rooms);
            
            // Initialize WebSocket manager
            console.log('Initializing WebSocket manager...');
            this.websocketManager = new WebSocketManager(this.robotController, this.rooms);
            
            // Set WebSocket manager reference in robot controller
            this.robotController.setWebSocketManager(this.websocketManager);
            
            // Start periodic status updates
            this.websocketManager.startStatusUpdates();
            
            // Update status to show robot is ready
            if (document.getElementById('status')) {
                document.getElementById('status').innerHTML = 'Mode: AI Mode<br>Camera: stabilized_follow<br>Status: Robot ready and positioned';
            }
            
            // Add ground
            console.log('Creating ground...');
            this.createGround();
            
            // Handle window resize
            window.addEventListener('resize', () => this.onWindowResize());
            
            console.log('Init complete!');
            
        } catch (error) {
            console.error('Error in init():', error);
            alert('Error in init: ' + error.message);
        }
    }
    
    setupLighting() {
        try {
            // Hemisphere + Ambient for natural fill
            const hemi = new THREE.HemisphereLight(0xb1e1ff, 0xb97a20, 0.45);
            this.scene.add(hemi);
            const ambient = new THREE.AmbientLight(0x404040, 0.35);
            this.scene.add(ambient);

            // Directional "sun" with soft shadows
            const sun = new THREE.DirectionalLight(0xffffff, 1.0);
            sun.position.set(60, 120, 60);
            sun.castShadow = true;
            sun.shadow.mapSize.set(2048, 2048);
            sun.shadow.radius = 2;          // PCF blur
            sun.shadow.normalBias = 0.5;    // reduce acne
            // Shadow camera frustum to cover most of the scene
            const cam = sun.shadow.camera;
            cam.left = -200; cam.right = 200; cam.top = 200; cam.bottom = -200;
            cam.near = 10; cam.far = 500;
            this.scene.add(sun);

            // Subtle exponential fog for depth (matches sky blue)
            if (!this.scene.fog) this.scene.fog = new THREE.FogExp2(0x87CEEB, 0.0022);

            console.log('Lighting setup complete');
        } catch (error) {
            console.error('Error in setupLighting:', error);
            throw error;
        }
    }
    
    createRobotCar() {
        try {
            console.log('Creating robot car...');
            this.robotCar = new THREE.Group();
            
            // Car body
            const bodyGeometry = new THREE.BoxGeometry(2, 1, 3);
            const bodyMaterial = new THREE.MeshStandardMaterial({ 
                color: 0x00ff00,
                roughness: 0.9,
                metalness: 0.0
            });
            const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
            body.position.y = 0.5;
            body.castShadow = true;
            this.robotCar.add(body);
            
            // Car wheels
            const wheelGeometry = new THREE.CylinderGeometry(0.5, 0.5, 0.3, 8);
            const wheelMaterial = new THREE.MeshStandardMaterial({ 
                color: 0x333333,
                roughness: 0.8,
                metalness: 0.1
            });
            
            const wheelPositions = [
                [-1, 0.5, -1.2], [1, 0.5, -1.2],
                [-1, 0.5, 1.2], [1, 0.5, 1.2]
            ];
            
            wheelPositions.forEach(pos => {
                const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
                wheel.position.set(...pos);
                wheel.rotation.z = Math.PI / 2;
                wheel.castShadow = true;
                this.robotCar.add(wheel);
            });
            
            // Car "eyes" (sensors)
            const eyeGeometry = new THREE.SphereGeometry(0.2, 8, 8);
            const eyeMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
            
            const leftEye = new THREE.Mesh(eyeGeometry, eyeMaterial);
            leftEye.position.set(-0.5, 1.2, 1);
            this.robotCar.add(leftEye);
            
            const rightEye = new THREE.Mesh(eyeGeometry, eyeMaterial);
            rightEye.position.set(0.5, 1.2, 1);
            this.robotCar.add(rightEye);
            
            // Set initial position in a known walkable corridor area
            // Position the robot in the center corridor, away from any obstacles
            this.robotCar.position.set(0, 0, 0); // Start in the center of the open field
            
            this.scene.add(this.robotCar);
            console.log('Robot car created and added to scene at position:', this.robotCar.position);
        } catch (error) {
            console.error('Error in createRobotCar:', error);
        }
    }
    
    createGround() {
        try {
            const groundGeometry = new THREE.PlaneGeometry(500, 500);
            // Procedural checker texture (no external assets)
            const size = 64, cells = 8;
            const canvas = document.createElement('canvas');
            canvas.width = size; canvas.height = size;
            const ctx = canvas.getContext('2d');
            for (let y = 0; y < cells; y++) {
                for (let x = 0; x < cells; x++) {
                    ctx.fillStyle = ((x + y) % 2 === 0) ? '#b8d8b8' : '#a4cfa4';
                    ctx.fillRect(x * (size / cells), y * (size / cells), (size / cells), (size / cells));
                }
            }
            const tex = new THREE.CanvasTexture(canvas);
            tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
            tex.repeat.set(50, 50);
            const groundMaterial = new THREE.MeshStandardMaterial({
                color: 0xffffff,
                map: tex,
                roughness: 0.95,
                metalness: 0.0
            });
            const ground = new THREE.Mesh(groundGeometry, groundMaterial);
            ground.rotation.x = -Math.PI / 2;
            ground.receiveShadow = true;
            this.scene.add(ground);
            console.log('Ground created and added to scene');
        } catch (error) {
            console.error('Error in createGround:', error);
            throw error;
        }
    }
    
    setupEventListeners() {
        try {
            // Keyboard controls
            document.addEventListener('keydown', (event) => {
                if (event.key === 'c' || event.key === 'C') { 
                    this.cycleCameraMode(); 
                    return; 
                }
                if (event.key === 'g' || event.key === 'G') { 
                    this.toggleGrid(); 
                    return; 
                }
                this.robotController.handleKeyDown(event.key);
            });
            
            document.addEventListener('keyup', (event) => {
                this.robotController.handleKeyUp(event.key);
            });
            
            // Mode switching
            document.getElementById('ai-mode').addEventListener('click', () => {
                this.setAIMode(true);
            });
            
            document.getElementById('manual-mode').addEventListener('click', () => {
                this.setAIMode(false);
            });
            
            // AI commands (single room for testing)
            document.getElementById('go-room-a').addEventListener('click', () => {
                if (this.websocketManager) {
                    this.websocketManager.addDestination('GEMC');
                }
            });
            
            document.getElementById('go-room-b').addEventListener('click', () => {
                if (this.websocketManager) {
                    this.websocketManager.addDestination('Fites');
                }
            });
            
            document.getElementById('go-room-c').addEventListener('click', () => {
                if (this.websocketManager) {
                    this.websocketManager.addDestination('SERF');
                }
            });
            
            document.getElementById('test-sequence').addEventListener('click', () => {
                if (this.websocketManager) {
                    // Add multiple destinations to test queue processing
                    this.websocketManager.addDestination('GEMC');
                    this.websocketManager.addDestination('Fites');
                    this.websocketManager.addDestination('SERF');
                    this.websocketManager.addDestination('iHub');
                }
            });
            
            document.getElementById('stop-ai').addEventListener('click', () => {
                this.stopAI();
            });
            
            document.getElementById('clear-queue').addEventListener('click', () => {
                if (this.websocketManager) {
                    this.websocketManager.clearDestinationQueue();
                }
            });
            
            // WebSocket controls
            document.getElementById('connect-ws').addEventListener('click', () => {
                this.connectWebSocket();
            });
            
            document.getElementById('disconnect-ws').addEventListener('click', () => {
                this.disconnectWebSocket();
            });
            
            console.log('Event listeners setup complete');
        } catch (error) {
            console.error('Error in setupEventListeners:', error);
        }
    }
    
    setAIMode(isAI) {
        this.robotController.setAIMode(isAI);
        
        // Update UI
        document.getElementById('ai-mode').classList.toggle('active', isAI);
        document.getElementById('manual-mode').classList.toggle('active', !isAI);
        
        // Update status
        this.updateStatus(this.robotController.getStatus());
    }
    
    goToRoom(roomKey) {
        const room = this.rooms[roomKey];
        if (room) {
            const status = this.robotController.goToRoom(roomKey, room.position);
            this.updateStatus(status);
        }
    }
    
    goToRoomSequence(roomKeys) {
        const status = this.robotController.goToRoomSequence(roomKeys, this.rooms);
        this.updateStatus(status);
    }
    
    stopAI() {
        const status = this.robotController.stopAI();
        this.updateStatus(status);
    }
    
    connectWebSocket() {
        try {
            if (this.websocketManager) {
                this.websocketManager.connect();
                this.updateStatus('WebSocket: Connecting to AI backend...');
            } else {
                this.updateStatus('WebSocket: Manager not initialized');
            }
        } catch (error) {
            this.updateStatus('WebSocket connection error: ' + error.message);
        }
    }
    
    disconnectWebSocket() {
        try {
            if (this.websocketManager) {
                this.websocketManager.disconnect();
                this.updateStatus('WebSocket: Disconnected from AI backend');
            }
        } catch (error) {
            this.updateStatus('WebSocket disconnect error: ' + error.message);
        }
    }
    
    updateStatus(message) {
        const status = document.getElementById('status');
        if (!status) { console.warn('Missing #status element; skipping status update'); return; }
        
        const currentMode = this.robotController.isAIMode ? 'AI Mode' : 'Manual Mode';
        const wsStatus = this.websocketManager ? (this.websocketManager.isConnected ? 'Connected' : 'Disconnected') : 'Not Initialized';
        const queueInfo = this.websocketManager ? `Queue: ${this.websocketManager.getDestinationQueue().length} destinations` : 'Queue: N/A';
        const targetInfo = this.robotController.aiTargets.length > 0 ? `<br>${this.robotController.getStatus()}` : '';
        
        status.innerHTML = `Mode: ${currentMode}<br>WebSocket: ${wsStatus}<br>${queueInfo}<br>Status: ${message}${targetInfo}`;
    }
    
    cycleCameraMode() {
        const order = ['stabilized_follow', 'chase', 'first_person'];
        const idx = order.indexOf(this.cameraMode);
        this.cameraMode = order[(idx + 1) % order.length];
        
        // Update status to show current camera mode
        const currentMode = this.robotController.isAIMode ? 'AI Mode' : 'Manual Mode';
        const targetInfo = this.robotController.aiTargets.length > 0 ? `<br>${this.robotController.getStatus()}` : '';
        const statusElement = document.getElementById('status');
        if (statusElement) {
            statusElement.innerHTML = `Mode: ${currentMode}<br>Camera: ${this.cameraMode}<br>Status: ${this.robotController.getStatus()}${targetInfo}`;
        }
        
        console.log(`Camera mode switched to: ${this.cameraMode}`);
    }
    
    toggleGrid() {
        if (!this.pathfinding) return;
        
        // Toggle grid visibility for debugging
        if (!this._gridHelper) {
            // Create grid helper
            this._gridHelper = new THREE.GridHelper(100, 100, 0x888888, 0xcccccc);
            this._gridHelper.position.y = 0.01; // Slightly above ground
            this._gridHelper.material.transparent = true;
            this._gridHelper.material.opacity = 0.3;
            this.scene.add(this._gridHelper);
            console.log('Grid helper enabled (press G again to hide)');
        } else {
            // Remove grid helper
            this.scene.remove(this._gridHelper);
            this._gridHelper = null;
            console.log('Grid helper disabled');
        }
    }
    
    animate() {
        try {
            const now = performance.now();
            this._lastTime = this._lastTime ?? now;
            const dt = Math.min(0.05, (now - this._lastTime) / 1000); // clamp 50ms
            this._lastTime = now;
            requestAnimationFrame(() => this.animate());
            
            if (this.robotCar && this.camera && this.renderer && this.scene) {
                // Update robot movement with deltaTime
                this.robotController.updateRobotMovement(dt);
                
                // Camera update by mode
                switch (this.cameraMode) {
                    case 'stabilized_follow': {
                        const desiredOffset = new THREE.Vector3(0, 30, 40); // world-space
                        const desiredPos = this.robotCar.position.clone().add(desiredOffset);
                        const camAlpha = 1 - Math.pow(0.0001, dt);
                        const lookAlpha = 1 - Math.pow(0.00005, dt);
                        
                        // look slightly toward next waypoint
                        let lookTarget = this.robotCar.position;
                        if (this.robotController && this.robotController.aiTargets &&
                            this.robotController.currentTargetIndex < this.robotController.aiTargets.length) {
                            const nextWP = this.robotController.aiTargets[this.robotController.currentTargetIndex];
                            if (nextWP) lookTarget = this.robotCar.position.clone().lerp(nextWP, 0.25);
                        }
                        
                        this._smoothCamPos.lerp(desiredPos, camAlpha);
                        this._smoothCamLook.lerp(lookTarget, lookAlpha);
                        
                        this.camera.position.copy(this._smoothCamPos);
                        this.camera.lookAt(this._smoothCamLook);
                        break;
                    }
                    case 'chase': {
                        const desiredLocal = new THREE.Vector3(0, 15, -25); // behind robot
                        if (!this._smoothCamQuat) this._smoothCamQuat = this.robotCar.quaternion.clone();
                        this._smoothCamQuat.slerp(this.robotCar.quaternion, Math.min(1, 4.0 * dt));
                        const desiredOffset = desiredLocal.clone().applyQuaternion(this._smoothCamQuat);
                        const desiredPos = this.robotCar.position.clone().add(desiredOffset);
                        const camAlpha = 1 - Math.pow(0.0001, dt);
                        const lookAlpha = 1 - Math.pow(0.00008, dt);
                        
                        const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(this._smoothCamQuat);
                        const lookTarget = this.robotCar.position.clone().add(fwd.multiplyScalar(10));
                        
                        this._smoothCamPos.lerp(desiredPos, camAlpha);
                        this._smoothCamLook.lerp(lookTarget, lookAlpha);
                        
                        this.camera.position.copy(this._smoothCamPos);
                        this.camera.lookAt(this._smoothCamLook);
                        break;
                    }
                    case 'first_person': {
                        const headLocal = new THREE.Vector3(0, 2, 0.2);
                        if (!this._smoothCamQuat) this._smoothCamQuat = this.robotCar.quaternion.clone();
                        this._smoothCamQuat.slerp(this.robotCar.quaternion, Math.min(1, 6.0 * dt));
                        
                        const camPos = this.robotCar.position.clone().add(headLocal.applyQuaternion(this._smoothCamQuat));
                        const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(this._smoothCamQuat);
                        const lookTarget = camPos.clone().add(fwd.multiplyScalar(6));
                        
                        this.camera.position.copy(camPos);
                        this.camera.lookAt(lookTarget);
                        break;
                    }
                }
                
                this.renderer.render(this.scene, this.camera);
            }
        } catch (error) {
            console.error('Error in animate:', error);
        }
    }
    
    onWindowResize() {
        if (this.camera && this.renderer) {
            this.camera.aspect = window.innerWidth / window.innerHeight;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(window.innerWidth, window.innerHeight);
        }
    }
}

// Initialize simulation when page loads
window.addEventListener('load', () => {
    console.log('Page loaded, checking for Three.js...');
    
    // Wait a bit for Three.js to load
    setTimeout(() => {
        if (typeof THREE !== 'undefined') {
            console.log('Three.js found, creating simulation...');
            try {
                window.robotSimulation = new RobotCarSimulation();
            } catch (error) {
                console.error('Failed to create simulation:', error);
                alert('Failed to create simulation: ' + error.message);
            }
        } else {
            console.error('Three.js not available after timeout');
            alert('Three.js not available after timeout!');
        }
    }, 1000);
});
