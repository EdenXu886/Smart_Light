import React from 'react';

function LightCard({ id, isOn, onToggle }) {
  return (
    <div style={{
      border: "1px solid #ccc",
      padding: "20px",
      borderRadius: "10px",
      width: "150px",
      textAlign: "center",
      backgroundColor: isOn ? "#c8f7c5" : "#f7c5c5"
    }}>
      <h3>Light {id}</h3>
      <p>Status: {isOn ? "ON" : "OFF"}</p>
      <button onClick={() => onToggle(id, 1)} style={{ marginRight: "5px" }}>
        Turn ON
      </button>
      <button onClick={() => onToggle(id, 0)}>
        Turn OFF
      </button>
    </div>
  );
}

export default LightCard;
