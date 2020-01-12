#include "LoadMenu.hpp"

TFormTextField::TFormTextField() {
    font = APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    color = nvgRGB(0xEF, 0xEF, 0xEF);
    multiline = false;
}

void TFormTextField::draw(const DrawArgs& args) {
    nvgScissor(args.vg, RECT_ARGS(args.clipBox));
    // Text
    if (font->handle >= 0) {
        bndSetFont(font->handle);

        NVGcolor highlightColor = color;
        highlightColor.a = 0.5;
        int begin = std::min(cursor, selection);
        int end = (this == APP->event->selectedWidget) ? std::max(cursor, selection) : -1;
        if (text.size() > 9) {
            text = text.substr(0, 9);
        }
        bndIconLabelCaret(args.vg, 0, -2,  box.size.x, box.size.y,
                          -1, color, 12, text.c_str(), highlightColor, begin, end);
        bndSetFont(APP->window->uiFont->handle);
    }

    nvgBeginPath(args.vg);
    nvgStrokeColor(args.vg, color);
    nvgStrokeWidth(args.vg, 1.0);
    nvgMoveTo(args.vg, 0, box.size.y);
    nvgLineTo(args.vg, box.size.x, box.size.y);
    nvgStroke(args.vg);

    nvgResetScissor(args.vg);
    Widget::draw(args);
}

std::string TFormTextField::getText() const {
    return text.substr(0, 9);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TFormNumberField::TFormNumberField() {
    font = APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    color = nvgRGB(0xEF, 0xEF, 0xEF);
    multiline = false;
    minimum = 1;
    range = 64;
    value = minimum;
}

void TFormNumberField::draw(const DrawArgs& args) {
    nvgScissor(args.vg, RECT_ARGS(args.clipBox));
    // Text
    if (font->handle >= 0) {
        bndSetFont(font->handle);

        NVGcolor highlightColor = color;
        highlightColor.a = 0.5;
        int begin = std::min(cursor, selection);
        int end = (this == APP->event->selectedWidget) ? std::max(cursor, selection) : -1;

        if (text.size() > 2) {
            text = text.substr(0, 2);
        }

        bndIconLabelCaret(args.vg, 0, -2,  box.size.x, box.size.y,
                          -1, color, 12, text.c_str(), highlightColor, begin, end);
        bndSetFont(APP->window->uiFont->handle);
    }

    nvgBeginPath(args.vg);
    nvgStrokeColor(args.vg, color);
    nvgStrokeWidth(args.vg, 1.0);
    nvgMoveTo(args.vg, 0, box.size.y);
    nvgLineTo(args.vg, box.size.x, box.size.y);
    nvgStroke(args.vg);

    nvgResetScissor(args.vg);
    Widget::draw(args);
}

void TFormNumberField::onDeselect(const event::Deselect& e) {
    updateText(text);
}

void TFormNumberField::onAction(const event::Action& e) {
    updateText(text);
}

void TFormNumberField::setValue(int newValue) {
    if (newValue >= minimum && newValue < (minimum + range)) {
        value = newValue;
        text = std::to_string(value);
        prevText = text;
    }
}

void TFormNumberField::updateText(const std::string& newText) {
    auto isNumeric = [](const std::string& str) -> bool {
        for(int i = 0; i < str.size(); ++i) {
            if(!std::isdigit(str[i])) {
                return false;
            }
        }
        return true;
    };

    if (!isNumeric(text) || text.size() < 1) {
        text = prevText;
        return;
    }
    prevText = text;

    int newValue = stoi(text);

    if (newValue >= minimum && newValue < (minimum + range)) {
        value = newValue;
    }
    else {
        text = std::to_string(value);
        prevText = text;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TFormLoadMenu::TFormLoadMenu() {
    box.size = Vec(238, 195);
    startWaveChoice = createWidget<TFormEditorNumberChoice>(Vec(buttonWidth + buttonOffset * 2, 20));
    startWaveChoice->range = 64;
    startWaveChoice->box.size.x = 28;
    startWaveChoice->box.size.y = 15;
    startWaveChoice->visible = false;
    addChild(startWaveChoice);

    endWaveChoice = createWidget<TFormEditorNumberChoice>(Vec(108, 20));
    endWaveChoice->range = 64;
    endWaveChoice->box.size.x = 28;
    endWaveChoice->box.size.y = 15;
    endWaveChoice->visible = false;
    addChild(endWaveChoice);

    ingestNewTable = [=](int startWave, int endWave) {
        onIngestTableCallback(*selectedBank, startWave, endWave, nameField->getText());
        hide();
    };

    auto triggerIngest = [=]() {
        ingestNewTable(*startWaveChoice->choice, *endWaveChoice->choice);
    };

    cancelButton = createNewMenuButton("Cancel", NULL, buttonWidth * 4 + buttonOffset * 5, 3, buttonWidth, buttonHeight);
    cancelButton->onClick = [=]() {
        hide();
    };
    addChild(cancelButton);
    confirmButton = createNewMenuButton("Okay", triggerIngest, buttonWidth * 4 + buttonOffset * 5, 21, buttonWidth, buttonHeight);
    addChild(confirmButton);

    waveDisplay = createWidget<TFormEditorWaveDisplay>(Vec(1.5,27));
    waveDisplay->box.size.x = box.size.x - 3.f;
    waveDisplay->box.size.y = box.size.y - 27.f;
    addChild(waveDisplay);
    waveSliderPos = 0.f;
    selectedWave = 0;

    waveLineColor = nvgRGB(0xAF, 0xAF, 0xAF);
    waveFillColor = nvgRGBA(0xAF, 0xAF, 0xAF, 0x6F);

    font = APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));

    nameFieldLabel = createWidget<PlainText>(Vec(buttonOffset, 5));
    nameFieldLabel->box.size.x = buttonWidth;
    nameFieldLabel->color = nvgRGB(0xEF, 0xEF, 0xEF);
    nameFieldLabel->size = 12;
    nameFieldLabel->horzAlignment = NVG_ALIGN_LEFT;
    nameFieldLabel->text = "Name:";
    addChild(nameFieldLabel);

    nameField = createWidget<TFormTextField>(Vec(buttonWidth + buttonOffset * 2, 3));
    nameField->box.size.x = buttonWidth * 2 + buttonOffset;
    nameField->box.size.y = buttonHeight;
    addChild(nameField);

    startWaveField = createWidget<TFormNumberField>(Vec(buttonOffset, 21));
    startWaveField->box.size.x = buttonWidth / 2;
    startWaveField->box.size.y = buttonHeight;
    startWaveField->setValue(1);
    addChild(startWaveField);

    endWaveField = createWidget<TFormNumberField>(Vec(buttonWidth + buttonOffset * 2, 21));
    endWaveField->box.size.x = buttonWidth / 2;
    endWaveField->box.size.y = buttonHeight;
    endWaveField->setValue(64);
    addChild(endWaveField);

    onView = [=]() {
        startWaveField->setValue(1);
        endWaveField->setValue(64);
    };
}

void TFormLoadMenu::draw(const DrawArgs& args) {
    std::string strDetectedWaves = "Found " + std::to_string(detectedWaves->size()) + " waves";
    nvgFillColor(args.vg, nvgRGB(0xEF, 0xEF, 0xEF));
    nvgFontFaceId(args.vg, font->handle);
    nvgTextLetterSpacing(args.vg, 0.0);

    nvgFontSize(args.vg, 12);
    nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
    //nvgText(args.vg, 5, 5, strDetectedWaves.c_str(), NULL);
    //nvgText(args.vg, 5, 21, "Start:", NULL);
    //nvgText(args.vg, 80, 21, "End:", NULL);

    // Horizontal bar
    nvgBeginPath(args.vg);
    nvgMoveTo(args.vg, 0, box.pos.y + 40);
    nvgLineTo(args.vg, box.size.x, box.pos.y + 40);
    nvgStrokeWidth(args.vg, 1.0);
    nvgStrokeColor(args.vg, nvgRGB(0xAF, 0xAF, 0xAF));
    nvgStroke(args.vg);

    Widget::draw(args);
}

void TFormLoadMenu::step() {
    if(detectedWaves) {
        for (unsigned long i = 0; i < (*detectedWaves).size(); ++i) {
            for (int j = 0; j < TFORM_MAX_WAVELENGTH; ++j) {
                waveDisplay->waveData[i][j] = 0.f;
            }
        }

        unsigned long k = 0;
        //for (unsigned long i = *startWaveChoice->choice; i <= *endWaveChoice->choice; ++i) {
        for (unsigned long i = startWaveField->value - 1; i < endWaveField->value; ++i) {
            for (int j = 0; j < TFORM_MAX_WAVELENGTH; ++j) {
                waveDisplay->waveData[k][j] = (*detectedWaves)[i][j];
            }
            ++k;
        }
    }

    //waveDisplay->numWaves = *endWaveChoice->choice - *startWaveChoice->choice + 1;
    waveDisplay->numWaves = endWaveField->value - (startWaveField->value - 1);
    selectedWave = waveDisplay->selectedWave;
    Widget::step();
}

void TFormLoadMenu::onDragMove(const event::DragMove& e) {
    waveDisplay->moveSliderPos(e.mouseDelta.y);
}
