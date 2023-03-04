package frc.robot.auton;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDashboardManager {
    private static final String SMARTDASHBOARD_PLACED_NODES_KEY = "placedNodes";
    private static final String SMARTDASHBOARD_SELECTED_NODE_KEY = "selectedNode";
    
    private Set<Double> placedNodes = new HashSet<Double>();
    private int selectedNode = 1;

    public AutoDashboardManager() {
        loadFromSmartDashboard();
    }

    public int getSelectedNode() {
        selectedNode = (int)Math.round(SmartDashboard.getNumber(SMARTDASHBOARD_SELECTED_NODE_KEY, selectedNode));
        return selectedNode;
    }

    public void notifyNodePlacement(int node) {
        placedNodes.add((double)node);
        SmartDashboard.putNumberArray(SMARTDASHBOARD_PLACED_NODES_KEY, placedNodes.stream().mapToDouble(d -> d).toArray());
    }

    private void loadFromSmartDashboard() {
        // placedNodes = Arrays.stream(SmartDashboard.getNumberArray(SMARTDASHBOARD_PLACED_NODES_KEY, new double[0])).collect(Collectors.toSet());
        placedNodes.clear();
        double[] placedArray = SmartDashboard.getNumberArray(SMARTDASHBOARD_PLACED_NODES_KEY, new double[0]);
        for(double placed : placedArray) {
            placedNodes.add(placed);
        }
        selectedNode = (int)Math.round(SmartDashboard.getNumber(SMARTDASHBOARD_SELECTED_NODE_KEY, selectedNode));
    }
}
