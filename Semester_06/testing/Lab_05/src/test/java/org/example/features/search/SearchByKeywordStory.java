package org.example.features.search;

import net.serenitybdd.junit.runners.SerenityRunner;
import net.thucydides.core.annotations.Issue;
import net.thucydides.core.annotations.Managed;
import net.thucydides.core.annotations.Steps;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.openqa.selenium.WebDriver;

import org.example.steps.serenity.EndUserSteps;

@RunWith(SerenityRunner.class)
public class SearchByKeywordStory {

    @Managed(uniqueSession = true)
    public WebDriver webdriver;

    @Steps
    public EndUserSteps anna;

    @Issue("#WIKI-1")
    @Test
    public void searching_by_keyword_apple_should_display_the_corresponding_article() {
        anna.is_the_home_page();
        anna.looks_for("apple");
        anna.should_see_definition("Rachel Appleby: Observations & Reflections: Learning from each other, learning from yourself");
    }

    @Issue("#WIKI-2")
    @Test
    public void searching_by_keyword_students_should_display_the_corresponding_article() {
        anna.is_the_home_page();
        anna.looks_for("students");
        anna.should_see_definition("students");
    }

    @Issue("#WIKI-3")
    @Test
    public void searching_by_keyword_research_should_display_the_corresponding_article() {
        anna.is_the_home_page();
        anna.looks_for("research");
        anna.should_see_definition("Research");
    }
}
