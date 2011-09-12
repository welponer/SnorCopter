package AeroQuad.AQ_Configurator.UI.ShipConnectionPanel;

import java.util.List;

public interface IShipConnectionPanelController
{
    void setPanel(AeroQuad.AQ_Configurator.UI.ShipConnectionPanel.IShipConnectionPanel connectionPanel);

    List<String> getComPortAvailable();

    void connect(String commPort, String speed);
}
