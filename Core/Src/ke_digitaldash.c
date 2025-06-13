/*
 * ke_digitaldash.c
 *
 *  Created on: Jun 8, 2025
 *      Author: Matth
 */

#include "ke_digitaldash.h"
#include "cJson.h"

bool digitaldash_to_json(const digitaldash *dash, char *buffer, size_t buffer_size) {
    cJSON *root = cJSON_CreateObject();

    if (!root) return false;

    // Serialize views
    cJSON *views = cJSON_AddArrayToObject(root, "view");
    for (int i = 0; i < MAX_VIEWS; i++) {
        cJSON *view = cJSON_CreateObject();
        cJSON_AddNumberToObject(view, "enabled", dash->view[i].enabled);
        cJSON_AddNumberToObject(view, "num_gauges", dash->view[i].num_gauges);
        cJSON_AddStringToObject(view, "background", view_background_string[dash->view[i].background]);

        cJSON *gauges = cJSON_AddArrayToObject(view, "gauge");
        for (int j = 0; j < MAX_GAUGES_PER_VIEW; j++) {
            cJSON *g = cJSON_CreateObject();
            cJSON_AddStringToObject(g, "theme", gauge_theme_string[dash->view[i].gauge[j].theme]);
            cJSON_AddNumberToObject(g, "pid", dash->view[i].gauge[j].pid->pid_uuid);
            cJSON_AddStringToObject(g, "units", "TODO");
            cJSON_AddItemToArray(gauges, g);
        }
        cJSON_AddItemToArray(views, view);
    }

    // Alerts
    cJSON *alerts = cJSON_AddArrayToObject(root, "alert");
    for (int i = 0; i < MAX_ALERTS; i++) {
        cJSON *a = cJSON_CreateObject();
        cJSON_AddNumberToObject(a, "enabled", dash->alert[i].enabled);
        cJSON_AddNumberToObject(a, "pid", dash->alert[i].pid->pid_uuid);
        cJSON_AddStringToObject(a, "compare", alert_comparison_string[dash->alert[i].compare]);
        cJSON_AddNumberToObject(a, "thresh", dash->alert[i].thresh);
        cJSON_AddStringToObject(a, "msg", dash->alert[i].msg);
        cJSON_AddItemToArray(alerts, a);
    }

    // Dynamic
    cJSON *dyn = cJSON_AddArrayToObject(root, "dynamic");
    for (int i = 0; i < MAX_DYNAMICS; i++) {
        cJSON *d = cJSON_CreateObject();
        cJSON_AddNumberToObject(d, "enabled", dash->dynamic[i].enabled);
        cJSON_AddNumberToObject(d, "view_index", dash->dynamic[i].view_index);
        cJSON_AddNumberToObject(d, "pid", dash->dynamic[i].pid->pid_uuid);
        cJSON_AddStringToObject(d, "compare", dynamic_comparison_string[dash->dynamic[i].compare]);
        cJSON_AddNumberToObject(d, "thresh", dash->dynamic[i].thresh);
        cJSON_AddStringToObject(d, "priority", dynamic_priority_string[dash->dynamic[i].priority]);
        cJSON_AddItemToArray(dyn, d);
    }

    cJSON_AddNumberToObject(root, "num_views", dash->num_views);

    // Print into user buffer
    bool success = cJSON_PrintPreallocated(root, buffer, buffer_size, /*format*/ 1);
    cJSON_Delete(root);
    return success;
}

bool json_to_digitaldash(const char* json_str, digitaldash* dash_out) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return false;

    // Views
    cJSON *views = cJSON_GetObjectItem(root, "view");
    for (int i = 0; i < MAX_VIEWS && i < cJSON_GetArraySize(views); i++) {
        cJSON *v = cJSON_GetArrayItem(views, i);
        dash_out->view[i].enabled = cJSON_GetObjectItem(v, "enabled")->valueint;
        dash_out->view[i].num_gauges = cJSON_GetObjectItem(v, "num_gauges")->valueint;

        cJSON *gauges = cJSON_GetObjectItem(v, "gauge");
        for (int j = 0; j < MAX_GAUGES_PER_VIEW && j < cJSON_GetArraySize(gauges); j++) {
            // You would map string PID and theme here
            // dash_out->view[i].gauge[j].pid = lookup_pid_by_name(...);
        }
    }

    // Alerts
    cJSON *alerts = cJSON_GetObjectItem(root, "alert");
    for (int i = 0; i < MAX_ALERTS && i < cJSON_GetArraySize(alerts); i++) {
        cJSON *a = cJSON_GetArrayItem(alerts, i);
        dash_out->alert[i].enabled = cJSON_GetObjectItem(a, "enabled")->valueint;
        dash_out->alert[i].compare = get_alert_compare_from_string((cJSON_GetObjectItem(a, "compare")->valuestring));
        dash_out->alert[i].thresh = (float)cJSON_GetObjectItem(a, "thresh")->valuedouble;
        strncpy(dash_out->alert[i].msg, cJSON_GetObjectItem(a, "msg")->valuestring, ALERT_MESSAGE_LEN - 1);
        dash_out->alert[i].msg[ALERT_MESSAGE_LEN - 1] = '\0';
    }

    // Dynamic
    cJSON *dyn = cJSON_GetObjectItem(root, "dynamic");
    for (int i = 0; i < MAX_DYNAMICS && i < cJSON_GetArraySize(dyn); i++) {
        cJSON *d = cJSON_GetArrayItem(dyn, i);
        dash_out->dynamic[i].enabled = cJSON_GetObjectItem(d, "enabled")->valueint;
        dash_out->dynamic[i].view_index = cJSON_GetObjectItem(d, "view_index")->valueint;
        dash_out->dynamic[i].compare = get_dynamic_compare_from_string(cJSON_GetObjectItem(d, "compare")->valuestring);
        dash_out->dynamic[i].thresh = (float)cJSON_GetObjectItem(d, "thresh")->valuedouble;
        // Similarly map priority string to enum here
    }

    dash_out->num_views = cJSON_GetObjectItem(root, "num_views")->valueint;

    cJSON_Delete(root);
    return true;
}
