#include "Valley.hpp"

// The pluginInstance-wide instance of the Plugin class
Plugin *pluginInstance;
Plugin *valleyPluginInstance;

void init(rack::Plugin *p) {
	pluginInstance = p;
	valleyPluginInstance = p;
    p->addModel(modelTopograph);
    p->addModel(modelUGraph);
    p->addModel(modelDexter);
    p->addModel(modelPlateau);
    p->addModel(modelInterzone);
    p->addModel(modelAmalgam);
    p->addModel(modelFeline);
    p->addModel(modelTerrorform);
}
