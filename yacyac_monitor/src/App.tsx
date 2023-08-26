import { Routes, Route, useLocation } from "react-router-dom";
import AppTemplate from "./templates/AppTemplate";
import Dashboard from "./pages/Dashboard";
import Historys from "./pages/Historys";
import Display from "./pages/Display";
import Containers from "./pages/Containers";

function App() {
  const location = useLocation();

  return (
    <div className="App">
      {location.pathname === "/display" ? (
        <Display />
      ) : (
        <AppTemplate>
          <Routes>
            <Route path="/" element={<Dashboard />}></Route>
            <Route path="/historys" element={<Historys />}></Route>
            <Route path="/containers" element={<Containers />}></Route>
          </Routes>
        </AppTemplate>
      )}
    </div>
  );
}

export default App;
