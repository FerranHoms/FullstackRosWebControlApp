import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import Welcome from './components/Welcome';
import ControlPanel from './components/ControlPanel';

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<Welcome />} />
        <Route path="/control" element={<ControlPanel />} />
      </Routes>
    </Router>
  );
}

export default App;