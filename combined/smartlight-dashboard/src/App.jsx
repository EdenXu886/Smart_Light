import React, { useState, useEffect } from 'react';
import axios from 'axios';
import LightCard from './components/LightCard';

const ESP32_URL = "http://192.168.1.100"; // Replace with your ESP32 IP

function App() {
  const [lux, setLux] = useState(0);
  const [lights, setLights] = useState([]);

  const fetchStatus = async () => {
    try {
      const response = await axios.get(`${ESP32_URL}/status`);
      setLux(response.data.lux);
      setLights(response.data.lights);
    } catch (error) {
      console.error("Failed to fetch status:", error);
    }
  };

  const toggleLight = async (lightId, newState) => {
    try {
      await axios.get(`${ESP32_URL}/toggle?light=${lightId}&state=${newState}`);
      fetchStatus();
    } catch (error) {
      console.error("Failed to toggle light:", error);
    }
  };

  useEffect(() => {
    fetchStatus();
    const interval = setInterval(fetchStatus, 2000);
    return () => clearInterval(interval);
  }, []);

  return (
    <div style={{ padding: "20px", fontFamily: "Arial" }}>
      <h1>Smart Light Dashboard</h1>
      <p>Ambient Light Level: {lux} lux</p>
      <div style={{ display: "flex", gap: "20px", marginTop: "20px" }}>
        {lights.map((state, idx) => (
          <LightCard key={idx} id={idx} isOn={state === 1} onToggle={toggleLight} />
        ))}
      </div>
    </div>
  );
}

export default App;
