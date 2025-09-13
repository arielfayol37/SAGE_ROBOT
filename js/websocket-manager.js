// WebSocket Manager for AI Backend Communication
class WebSocketManager {
    constructor(robotController, rooms) {
        this.robotController = robotController;
        this.rooms = rooms;
        this.websocket = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 2000; // 2 seconds
        
        // Destination queue management
        this.destinationQueue = [];
        this.isQueueProcessing = false;
        
        // Message types for communication
        this.messageTypes = {
            ROUTE_UPDATE: 'route_update',
            ROUTE_CANCEL: 'route_cancel',
            ROBOT_STATUS: 'robot_status',
            ROBOT_ARRIVAL: 'robot_arrival',
            ROBOT_ERROR: 'robot_error',
            HEARTBEAT: 'heartbeat',
            ACKNOWLEDGMENT: 'acknowledgment',
            CONNECTION_ESTABLISHED: 'connection_established'
        };
        
        console.log('WebSocketManager initialized');
    }
    
    // Connect to AI backend
    connect() {
        try {
            // Replace with your FastAPI backend WebSocket endpoint
            const wsUrl = 'ws://localhost:8002/ws/robot';
            
            console.log('Attempting to connect to:', wsUrl);
            this.websocket = new WebSocket(wsUrl);
            
            this.websocket.onopen = () => {
                console.log('WebSocket connected to AI backend');
                this.isConnected = true;
                this.reconnectAttempts = 0;
                
                // Immediately update connection status
                this.updateConnectionStatus('Connected to AI Backend');
                
                // Send initial robot status
                this.sendMessage({
                    type: this.messageTypes.ROBOT_STATUS,
                    data: {
                        status: 'connected',
                        position: this.robotController.robotCar.position,
                        mode: this.robotController.isAIMode ? 'ai' : 'manual'
                    }
                });
                
                // Start status updates
                this.startStatusUpdates();
            };
            
            this.websocket.onmessage = (event) => {
                this.handleIncomingMessage(event.data);
            };
            
            this.websocket.onclose = (event) => {
                console.log('WebSocket disconnected:', event.code, event.reason);
                this.isConnected = false;
                this.updateConnectionStatus('Disconnected from AI Backend');
                
                // Attempt reconnection if not manually disconnected
                if (event.code !== 1000) { // 1000 = normal closure
                    this.attemptReconnection();
                }
            };
            
            this.websocket.onerror = (error) => {
                console.error('WebSocket error:', error);
                this.updateConnectionStatus('Connection Error');
            };
            
        } catch (error) {
            console.error('Failed to create WebSocket connection:', error);
            this.updateConnectionStatus('Connection Failed: ' + error.message);
        }
    }
    
    // Disconnect from AI backend
    disconnect() {
        if (this.websocket) {
            this.websocket.close(1000, 'Manual disconnect');
            this.websocket = null;
            this.isConnected = false;
            this.updateConnectionStatus('Disconnected');
        }
        
        // Stop status updates
        this.stopStatusUpdates();
    }
    
    // Attempt to reconnect
    attemptReconnection() {
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.log('Max reconnection attempts reached');
            this.updateConnectionStatus('Reconnection Failed');
            return;
        }
        
        this.reconnectAttempts++;
        console.log(`Attempting reconnection ${this.reconnectAttempts}/${this.maxReconnectAttempts}...`);
        
        setTimeout(() => {
            this.connect();
        }, this.reconnectDelay * this.reconnectAttempts);
    }
    
    // Send message to AI backend
    sendMessage(message) {
        if (this.websocket && this.isConnected) {
            try {
                this.websocket.send(JSON.stringify(message));
                console.log('Sent message to AI backend:', message);
            } catch (error) {
                console.error('Failed to send message:', error);
            }
        } else {
            console.warn('WebSocket not connected, cannot send message');
        }
    }
    
    // Handle incoming messages from AI backend
    handleIncomingMessage(data) {
        try {
            const message = JSON.parse(data);
            console.log('Received message from AI backend:', message);
            
            switch (message.type) {
                case this.messageTypes.ROUTE_UPDATE:
                    this.handleRouteUpdate(message.data);
                    break;
                    
                case this.messageTypes.ROUTE_CANCEL:
                    this.handleRouteCancel(message.data);
                    break;
                    
                case this.messageTypes.ROBOT_STATUS:
                    this.handleRobotStatus(message.data);
                    break;
                    
                case this.messageTypes.ROBOT_ARRIVAL:
                    this.handleRobotArrival(message.data);
                    break;
                    
                case this.messageTypes.ROBOT_ERROR:
                    this.handleRobotError(message.data);
                    break;
                    
                case this.messageTypes.HEARTBEAT:
                    this.handleHeartbeat(message.data);
                    break;
                    
                case this.messageTypes.ACKNOWLEDGMENT:
                    this.handleAcknowledgment(message.data);
                    break;
                    
                case this.messageTypes.CONNECTION_ESTABLISHED:
                    this.handleConnectionEstablished(message.data);
                    break;
                    
                default:
                    console.log('Unknown message type:', message.type);
            }
            
        } catch (error) {
            console.error('Error handling incoming message:', error);
        }
    }
    
    // Handle route update from AI backend
    handleRouteUpdate(data) {
        const { destinations, clearExisting = false } = data;
        
        console.log(`AI Backend route update: ${destinations.length} destinations`);
        
        if (clearExisting) {
            this.destinationQueue = [];
            // FORCE STOP: Tell robot controller to stop current movement immediately
            this.robotController.stopAI();
            
            // Reset queue processing state to allow immediate processing
            this.isQueueProcessing = false;
            
            console.log('Cleared existing destinations and stopped robot movement');
        }
        
        // Add new destinations to the queue
        this.destinationQueue.push(...destinations);
        
        console.log(`Updated destination queue: ${this.destinationQueue.join(' → ')}`);
        
        // Update UI to reflect queue change
        this.updateQueueStatus();
        
        // Start processing immediately
        this.processDestinationQueue();
    }
    
    // Handle route cancellation from AI backend
    handleRouteCancel(data) {
        const { destination } = data;
        
        if (destination) {
            // Cancel specific destination
            const index = this.destinationQueue.indexOf(destination);
            if (index !== -1) {
                this.destinationQueue.splice(index, 1);
                console.log(`Cancelled destination: ${destination}`);
            }
        } else {
            // Cancel all destinations
            this.destinationQueue = [];
            console.log('Cancelled all destinations');
        }
        
        // Update robot controller
        this.updateRobotController();
    }
    
    // Handle robot status request from AI backend
    handleRobotStatus(data) {
        // Send current robot status
        this.sendRobotStatus();
    }
    
    // Handle robot arrival notification
    handleRobotArrival(data) {
        console.log('Robot arrival notification:', data);
        
        // Send arrival notification to AI backend with actual room name
        if (data.destination) {
            this.sendMessage({
                type: this.messageTypes.ROBOT_ARRIVAL,
                data: {
                    destination: data.destination,  // Use actual room name
                    position: this.robotController.robotCar.position,
                    timestamp: new Date().toISOString()
                }
            });
        }
    }
    
    // Handle robot error notification
    handleRobotError(data) {
        console.log('Robot error notification:', data);
    }
    
    // Handle heartbeat
    handleHeartbeat(data) {
        // Send heartbeat response
        this.sendMessage({
            type: this.messageTypes.HEARTBEAT,
            data: { timestamp: new Date().toISOString() }
        });
    }
    
    // Handle acknowledgment messages
    handleAcknowledgment(data) {
        console.log('Received acknowledgment for:', data.message_type);
        // Update connection status to show we're fully connected
        if (this.isConnected) {
            this.updateConnectionStatus('Connected to AI Backend');
        }
    }
    
    // Handle connection established message
    handleConnectionEstablished(data) {
        console.log('Connection established with robot ID:', data.robot_id);
        this.updateConnectionStatus('Connected to AI Backend');
        // Start sending status updates
        this.startStatusUpdates();
    }
    
    // Add destination to queue (for manual testing)
    addDestination(destination) {
        // Clear existing destinations and stop current movement when adding new ones
        this.destinationQueue = [];
        this.robotController.stopAI();
        
        // Reset queue processing state to allow immediate processing
        this.isQueueProcessing = false;
        
        console.log('Cleared existing destinations and stopped robot movement');
        
        // Add new destination
        this.destinationQueue.push(destination);
        console.log(`Added destination: ${destination}`);
        console.log(`Current queue: ${this.destinationQueue.join(' → ')}`);
        
        // Update UI to reflect queue change
        this.updateQueueStatus();
        
        // Start processing immediately
        this.processDestinationQueue();
    }
    
    // Clear destination queue
    clearDestinationQueue() {
        this.destinationQueue = [];
        console.log('Destination queue cleared');
        
        // Update UI to reflect queue change
        this.updateQueueStatus();
        
        this.updateRobotController();
    }
    
    // Process destination queue
    async processDestinationQueue() {
        if (this.destinationQueue.length === 0) {
            this.isQueueProcessing = false;
            return;
        }
        
        this.isQueueProcessing = true;
        
        while (this.destinationQueue.length > 0) {
            const destination = this.destinationQueue[0];
            console.log(`Processing destination: ${destination}`);
            
            // Check if room exists
            if (this.rooms[destination]) {
                // Send to robot controller
                const status = this.robotController.goToRoom(destination, this.rooms[destination].position);
                console.log(`Robot controller response: ${status}`);
                
                // Wait for robot to reach destination
                await this.waitForDestinationReached(destination);
                
                // Remove from queue
                this.destinationQueue.shift();
                console.log(`Completed destination: ${destination}`);
                
                // Update UI to reflect queue change
                this.updateQueueStatus();
                
            } else {
                console.error(`Room not found: ${destination}`);
                this.destinationQueue.shift(); // Remove invalid destination
            }
        }
        
        this.isQueueProcessing = false;
        console.log('Destination queue processing complete');
    }
    
    // Wait for robot to reach destination
    waitForDestinationReached(destination) {
        return new Promise((resolve) => {
            const checkInterval = setInterval(() => {
                // Check if robot has reached the destination
                if (this.robotController.aiTargets.length === 0) {
                    clearInterval(checkInterval);
                    resolve();
                }
            }, 100);
            
            // Timeout after 30 seconds
            setTimeout(() => {
                clearInterval(checkInterval);
                console.warn(`Timeout waiting for destination: ${destination}`);
                resolve();
            }, 30000);
        });
    }
    
    // Update robot controller with current queue
    updateRobotController() {
        // This method can be used to sync the queue with the robot controller
        console.log('Updating robot controller with queue:', this.destinationQueue);
    }
    
    // Send robot status to AI backend
    sendRobotStatus() {
        const status = {
            timestamp: new Date().toISOString(),
            position: {
                x: this.robotController.robotCar.position.x,
                y: this.robotController.robotCar.position.y,
                z: this.robotController.robotCar.position.z
            },
            rotation: this.robotController.robotCar.rotation.y,
            mode: this.robotController.isAIMode ? 'ai' : 'manual',
            status: this.robotController.aiTargets.length > 0 ? 'moving' : 'idle',
            destinations: this.destinationQueue,
            current_target: this.robotController.aiTargets.length > 0 ? 
                this.robotController.aiTargets[this.robotController.currentTargetIndex] : null,
            ai_targets: this.robotController.aiTargets.length,
            current_target_index: this.robotController.currentTargetIndex
        };
        
        this.sendMessage({
            type: this.messageTypes.ROBOT_STATUS,
            data: status
        });
    }
    
    // Start periodic status updates
    startStatusUpdates() {
        // Send status every 2 seconds
        this.statusUpdateInterval = setInterval(() => {
            if (this.isConnected) {
                this.sendRobotStatus();
            }
        }, 2000);
    }
    
    // Stop periodic status updates
    stopStatusUpdates() {
        if (this.statusUpdateInterval) {
            clearInterval(this.statusUpdateInterval);
            this.statusUpdateInterval = null;
        }
    }
    
    // Get current destination queue
    getDestinationQueue() {
        return this.destinationQueue;
    }
    
    // Update queue status in UI
    updateQueueStatus() {
        const statusDiv = document.getElementById('status');
        if (statusDiv) {
            const lines = statusDiv.innerHTML.split('<br>');
            
            // Update the queue line
            const queueStatus = this.destinationQueue.length > 0 ? 
                `${this.destinationQueue.join(' → ')}` : 'Empty';
            for (let i = 0; i < lines.length; i++) {
                if (lines[i].includes('Queue:')) {
                    lines[i] = `Queue: ${queueStatus}`;
                    break;
                }
            }
            
            statusDiv.innerHTML = lines.join('<br>');
        }
    }
    
    // Update connection status in UI
    updateConnectionStatus(status) {
        console.log('Updating connection status:', status);
        
        // Update the main simulation status if available
        if (window.robotSimulation && window.robotSimulation.updateStatus) {
            window.robotSimulation.updateStatus(`WebSocket: ${status}`);
        }
        
        // Also update the status div directly if the simulation isn't ready
        const statusDiv = document.getElementById('status');
        if (statusDiv) {
            const lines = statusDiv.innerHTML.split('<br>');
            
            // Update the WebSocket line
            for (let i = 0; i < lines.length; i++) {
                if (lines[i].includes('WebSocket:')) {
                    lines[i] = `WebSocket: ${status}`;
                    break;
                }
            }
            
            // Update the queue line
            const queueStatus = this.destinationQueue.length > 0 ? 
                `${this.destinationQueue.join(' → ')}` : 'Empty';
            for (let i = 0; i < lines.length; i++) {
                if (lines[i].includes('Queue:')) {
                    lines[i] = `Queue: ${queueStatus}`;
                    break;
                }
            }
            
            statusDiv.innerHTML = lines.join('<br>');
        }
    }
}

