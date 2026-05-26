# Layout for V3

## Top bar
* Shows Company (ROV) in bright blue letters, left aligned (same as v2)
* shows product name and version number in smaller, gray letter to the right of company name, left aligned (same as v2)
* remove 'statis UI concept' in right corner
* text displaying last requests' success state, right aligned (same as v2)
* refresh button for status moved here, right aligned, to the left of success text

## Status section
* section lies below top bar (same as v2)
* displays mapper state, mapping state, current topics, current action, and save state
* each status box should only be big enough to display the largest text it will display (e.g. "active" for the mapper status, see requests.md for more) - limit topics displayed to 32 characters (including commas and spaces)

## Action setup + queue
* change name to "Action Wizard"
* move to left side of screen (right of actication section, see details on that)
* action name + type move to below the preset action drop down, right-aligned
* each field of the action should be the same size; right now, the name and type fields are different sizes
* make add to queue and map now buttons same color as deactivate mapper button

## Activation section
* change to have a shape similar to action setup (more like a column)
* has text box for user to type in topic names, one topic per line (same as v2)
* has button below text box; when inactive, button is green and reads "Activate"; when active, button is red and reads "Deactivate"

## Completed Mappings + toggle mapping
* similar size and shape to action setup
* at top of section, have button similar to the one outlined in the activation section above, except the text is "Start Mapping" and "Stop Mapping"
* after pressing stop mapping button and recieved successful mapping response from server, add finished mapping to list of completed mappings (similar to the action queue)
* below the list of competed mappings, add "save mappings" button (request not implemented yet)
* after save amppings button is pressed, list of completed mappings is cleared

## Visual
* this gives a little bit of a visual, but isn't detailed

```txt
|ROV JOYSTICK MAPPER V3
|Mapper| |Mapping| |Topics| |Current Action| |Save State|  |refresh| |last msg status|
|
|Activation|               |Action Wizard|              |Mapping Wizard|
|                          |Preset Action|
|topic text box|           |action name|/|action type|  |Start/Stop Mapping|
|                          |ROS action str:|            |List of completed|
|Activate/Deactivate|      |Add to Queue| |Map Now|
|                          |Action Queue|               
|
|                          |Map Next Action|            |Save Completed Mappings|
|
|Request Console|
|
|
```