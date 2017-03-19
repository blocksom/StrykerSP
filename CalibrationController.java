/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package hip.positioning.system;

import java.net.URL;
import java.util.ResourceBundle;
import javafx.event.*;
import javafx.fxml.*;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.image.*;
import javafx.scene.layout.*;
import javafx.scene.text.TextAlignment;

/**
 * FXML Controller class
 *
 * @author Tristan
 */
public class CalibrationController implements Initializable {
    int nextStep = 0;
    @FXML
    Label stepLabel;
    
    @FXML
    Label directions;
    
    @FXML
    ImageView imageView;
    
//    @FXML
//    Image image = new Image("..\\..\\..\\newpackage\\step1.png");
    
    @FXML 
    private void goToNextStep(ActionEvent event) {
        if (nextStep == 0) {
            stepLabel.setText("Step 2");
            directions.setText("With brace removed from patient, close brace to"
            + " most closed position.");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
        }
        else if (nextStep == 1) {
            stepLabel.setText("Step 3");
            directions.setText("Place brace on patient");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
        }
        else if (nextStep == 2) {
            stepLabel.setText("Step 4");
            directions.setText("With brace properly attached to patient, place "
                + "patient such that the hip extension angle is zero degrees");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
        }
        else if (nextStep == 3) {
            stepLabel.setText("Step 5");
            directions.setText("With brace properly attached to patient, place "
                + "patient such that the abduction angle is zero degrees.");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
        }
        else if (nextStep == 4) {
            stepLabel.setText("Calibration Completed!");
            directions.setText("");
        }
        nextStep++;
    }
    
    /**
     * Initializes the controller class.
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) {
        // TODO
        stepLabel.setText("Step 1");
            directions.setText("With brace removed from patient, open brace to"
            + " most extended position.");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
            
    }    
    
}
