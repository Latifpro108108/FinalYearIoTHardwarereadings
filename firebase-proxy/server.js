require('dotenv').config();
const express = require('express');
const axios = require('axios');
const cors = require('cors');

const app = express();
const port = process.env.PORT || 3000;

// Middleware
app.use(cors());
app.use(express.json());
app.use((req, res, next) => {
    console.log('Incoming request:', {
        method: req.method,
        path: req.path,
        headers: req.headers,
        body: req.body
    });
    next();
});

// Firebase configuration
const FIREBASE_URL = process.env.FIREBASE_DATABASE_URL || 'https://mxchipreads-default-rtdb.firebaseio.com';
const API_KEY = process.env.FIREBASE_API_KEY || 'AIzaSyArAhSbTt21GvBamRg20XDwiczccv6oZ5Q';

console.log('Server configuration:', {
    FIREBASE_URL,
    API_KEY: API_KEY ? '***' + API_KEY.slice(-6) : 'not set'
});

// Proxy endpoint for sensor readings
app.post('/sensor-data', async (req, res) => {
    try {
        console.log('Raw request body:', req.body);
        
        // Ensure data is properly formatted
        const temperature = parseFloat(req.body.temperature);
        const humidity = parseFloat(req.body.humidity);
        const timestamp = parseInt(req.body.timestamp);

        if (isNaN(temperature) || isNaN(humidity) || isNaN(timestamp)) {
            throw new Error('Invalid data format');
        }

        const data = {
            temperature,
            humidity,
            timestamp,
            received_at: new Date().toISOString()
        };

        console.log('Processed data:', data);
        console.log('Sending to Firebase URL:', `${FIREBASE_URL}/sensorReadings.json?auth=${API_KEY}`);
        
        // Forward the data to Firebase
        const response = await axios({
            method: 'POST',
            url: `${FIREBASE_URL}/sensorReadings.json`,
            params: {
                auth: API_KEY
            },
            data: data,
            headers: {
                'Content-Type': 'application/json'
            }
        });

        console.log('Firebase response:', response.data);

        res.json({
            success: true,
            data: response.data
        });
    } catch (error) {
        console.error('Error details:', {
            name: error.name,
            message: error.message,
            response: error.response ? {
                status: error.response.status,
                statusText: error.response.statusText,
                data: error.response.data
            } : 'No response data',
            config: error.config ? {
                url: error.config.url,
                method: error.config.method,
                headers: error.config.headers,
                data: error.config.data
            } : 'No config data'
        });

        res.status(500).json({
            success: false,
            error: 'Failed to send data to Firebase',
            details: error.message
        });
    }
});

// Health check endpoint
app.get('/health', (req, res) => {
    res.json({ status: 'healthy' });
});

// Test endpoint for Firebase connection
app.get('/test-firebase', async (req, res) => {
    try {
        const testData = {
            test: true,
            timestamp: Date.now()
        };
        
        const response = await axios({
            method: 'POST',
            url: `${FIREBASE_URL}/test.json`,
            params: {
                auth: API_KEY
            },
            data: testData
        });

        res.json({
            success: true,
            data: response.data
        });
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            details: error.response ? error.response.data : null
        });
    }
});

// Log when server starts
app.listen(port, () => {
    console.log(`Proxy server running on port ${port}`);
    console.log(`Firebase URL: ${FIREBASE_URL}`);
}); 