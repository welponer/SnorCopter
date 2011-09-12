package AeroQuad.AQ_Configurator.UI.MotorsPanel;

import AeroQuad.AQ_Configurator.SerialCommunicator.ISerialCommunicator;
import AeroQuad.AQ_Configurator.SerialCommunicator.Message.FlightDataMessage;
import AeroQuad.AQ_Configurator.SerialCommunicator.Message.SerialMessage;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

public class MotorsPanelController implements IMotorsPanelController
{
    private IMotorsPanel _panel;

    public MotorsPanelController(ISerialCommunicator communicator)
    {
        communicator.addListener(SerialMessage.FLIGHT_DATA_MESSAGE_NAME, new PropertyChangeListener()
        {
            @Override
            public void propertyChange(PropertyChangeEvent evt)
            {
                final FlightDataMessage message = (FlightDataMessage)evt.getNewValue();
                processMessage(message);
            }
        });        
    }

    private void processMessage(final FlightDataMessage message)
    {
        _panel.setMotorValue(0,message.getMotorThrottle(0));
        _panel.setMotorValue(1,message.getMotorThrottle(1));
        _panel.setMotorValue(2,message.getMotorThrottle(2));
        _panel.setMotorValue(3,message.getMotorThrottle(3));
    }

    @Override
    public void setPanel(final IMotorsPanel motorsPanel)
    {
        _panel = motorsPanel;
    }

}
