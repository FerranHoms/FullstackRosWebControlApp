import React, { useState } from 'react';
import { controlSolenoid, controlPump } from '../api';

const ControlPanel = () => {
  const [solenoidNumber, setSolenoidNumber] = useState(1);
  const [pumpNumber, setPumpNumber] = useState(1);
  const [durationSeconds, setDurationSeconds] = useState(0);
  const [solenoidAction, setSolenoidAction] = useState('timed');
  const [pumpAction, setPumpAction] = useState('timed');

  const handleSolenoidSubmit = async (e) => {
    e.preventDefault();
    const data = {
      solenoid_number: solenoidNumber,
      duration_seconds: solenoidAction === 'timed' ? durationSeconds : 0,
      start: solenoidAction === 'start',
      stop: solenoidAction === 'stop',
    };
    try {
      await controlSolenoid(data);
      alert('Solenoid command sent successfully');
    } catch (error) {
      alert('Failed to send solenoid command');
    }
  };

  const handlePumpSubmit = async (e) => {
    e.preventDefault();
    const data = {
      pump_number: pumpNumber,
      duration_seconds: pumpAction === 'timed' ? durationSeconds : 0,
      start: pumpAction === 'start',
      stop: pumpAction === 'stop',
    };
    try {
      await controlPump(data);
      alert('Pump command sent successfully');
    } catch (error) {
      alert('Failed to send pump command');
    }
  };

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-gray-100 p-4">
      <h1 className="text-3xl font-bold mb-6 text-gray-800">Control Panel</h1>
      
      {/* Solenoid Controls */}
      <form onSubmit={handleSolenoidSubmit} className="mb-8 w-full max-w-md">
        <h2 className="text-2xl font-semibold mb-4">Solenoids</h2>
        <div className="mb-4">
          <label className="block text-gray-700">Solenoid Number (1-4):</label>
          <input
            type="number"
            min="1"
            max="4"
            value={solenoidNumber}
            onChange={(e) => setSolenoidNumber(parseInt(e.target.value))}
            className="border border-gray-300 p-2 rounded w-full"
          />
        </div>
        <div className="mb-4">
          <label className="block text-gray-700">Action:</label>
          <select
            value={solenoidAction}
            onChange={(e) => setSolenoidAction(e.target.value)}
            className="border border-gray-300 p-2 rounded w-full"
          >
            <option value="timed">Timed Activation</option>
            <option value="start">Start (Indefinite)</option>
            <option value="stop">Stop</option>
          </select>
        </div>
        {solenoidAction === 'timed' && (
          <div className="mb-4">
            <label className="block text-gray-700">Duration (seconds):</label>
            <input
              type="number"
              min="0"
              value={durationSeconds}
              onChange={(e) => setDurationSeconds(parseInt(e.target.value))}
              className="border border-gray-300 p-2 rounded w-full"
            />
          </div>
        )}
        <button
          type="submit"
          className="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded w-full"
        >
          Send Solenoid Command
        </button>
      </form>

      {/* Pump Controls */}
      <form onSubmit={handlePumpSubmit} className="w-full max-w-md">
        <h2 className="text-2xl font-semibold mb-4">Pumps</h2>
        <div className="mb-4">
          <label className="block text-gray-700">Pump Number (1-4):</label>
          <input
            type="number"
            min="1"
            max="4"
            value={pumpNumber}
            onChange={(e) => setPumpNumber(parseInt(e.target.value))}
            className="border border-gray-300 p-2 rounded w-full"
          />
        </div>
        <div className="mb-4">
          <label className="block text-gray-700">Action:</label>
          <select
            value={pumpAction}
            onChange={(e) => setPumpAction(e.target.value)}
            className="border border-gray-300 p-2 rounded w-full"
          >
            <option value="timed">Timed Activation</option>
            <option value="start">Start (Indefinite)</option>
            <option value="stop">Stop</option>
          </select>
        </div>
        {pumpAction === 'timed' && (
          <div className="mb-4">
            <label className="block text-gray-700">Duration (seconds):</label>
            <input
              type="number"
              min="0"
              value={durationSeconds}
              onChange={(e) => setDurationSeconds(parseInt(e.target.value))}
              className="border border-gray-300 p-2 rounded w-full"
            />
          </div>
        )}
        <button
          type="submit"
          className="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded w-full"
        >
          Send Pump Command
        </button>
      </form>
    </div>
  );
};

export default ControlPanel;