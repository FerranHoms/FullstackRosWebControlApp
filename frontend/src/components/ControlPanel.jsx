import React, { useState } from 'react';
import { controlPump } from '../api';

const CoffeeDispenser = () => {
  const [pumpNumber, setPumpNumber] = useState(1);
  const [durationSeconds, setDurationSeconds] = useState(0);
  const [pumpAction, setPumpAction] = useState('timed');

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
      alert('Coffee brew command sent successfully');
    } catch (error) {
      alert('Failed to send coffee brew command');
    }
  };

  return (
    <div className="flex flex-col items-center justify-center min-h-screen w-full bg-gradient-to-br from-yellow-100 to-orange-200 p-4">
      <h1 className="text-4xl font-bold mb-8 text-orange-700">☕ Coffee Dispenser ☕</h1>
      <form onSubmit={handlePumpSubmit} className="w-full max-w-md bg-white shadow-md rounded px-8 pt-6 pb-8">
        <h2 className="text-2xl font-semibold mb-4 text-gray-800">Brew Your Coffee</h2>
        <div className="mb-4">
          <label className="block text-gray-700">Dispenser Number (1-2):</label>
          <input
            type="number"
            min="1"
            max="2"
            value={pumpNumber}
            onChange={(e) => setPumpNumber(parseInt(e.target.value))}
            className="border border-gray-300 p-2 rounded w-full text-gray-800"
          />
        </div>
        <div className="mb-4">
          <label className="block text-gray-700">Brew Action:</label>
          <select
            value={pumpAction}
            onChange={(e) => setPumpAction(e.target.value)}
            className="border border-gray-300 p-2 rounded w-full text-gray-800"
          >
            <option value="timed">Timed Brew</option>
            <option value="start">Start Brewing (Indefinite)</option>
            <option value="stop">Stop Brewing</option>
          </select>
        </div>
        {pumpAction === 'timed' && (
          <div className="mb-4">
            <label className="block text-gray-700">Brew Duration (seconds):</label>
            <input
              type="number"
              min="0"
              value={durationSeconds}
              onChange={(e) => setDurationSeconds(parseInt(e.target.value))}
              className="border border-gray-300 p-2 rounded w-full text-gray-800"
            />
          </div>
        )}
        <button
          type="submit"
          className="bg-amber-600 hover:bg-amber-800 text-white font-bold py-2 px-4 rounded w-full"
        >
          Brew Coffee
        </button>
      </form>
    </div>
  );
};

export default CoffeeDispenser;
