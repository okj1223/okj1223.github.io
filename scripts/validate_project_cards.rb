#!/usr/bin/env ruby
# frozen_string_literal: true

require "date"
require "pathname"
require "set"
require "stringio"
require "yaml"

ROOT = Pathname(__dir__).join("..").realpath
PROJECTS_DIR = ROOT.join("_projects")
PROJECT_CARDS_PATH = ROOT.join("_data", "project_cards.yml")
ALLOWED_FILTERS = %w[robotics ai control environmental mechanical].freeze

def blank?(value)
  return true if value.nil?
  return value.strip.empty? if value.is_a?(String)
  return value.empty? if value.respond_to?(:empty?)

  false
end

def present?(value)
  !blank?(value)
end

def load_yaml_file(path, context)
  YAML.safe_load(path.read, permitted_classes: [Date, Time], aliases: true)
rescue Psych::Exception => e
  raise "#{context}: invalid YAML (#{e.message})"
end

def load_front_matter(path)
  contents = path.read
  match = contents.match(/\A---\s*\n(.*?)\n---\s*\n/m)
  raise "#{path.basename}: missing YAML front matter" unless match

  load_yaml_file(StringIO.new(match[1]), path.basename.to_s) || {}
end

def normalized_array(value)
  Array(value).flatten.compact.reject { |item| blank?(item) }
end

def add_error(errors, message)
  errors << message
end

projects = {}
errors = []

Dir[PROJECTS_DIR.join("*.md")].sort.each do |file|
  path = Pathname(file)
  front_matter = load_front_matter(path)
  permalink = front_matter["permalink"]

  if blank?(permalink)
    add_error(errors, "#{path.basename}: missing `permalink` in front matter")
    next
  end

  if projects.key?(permalink)
    add_error(errors, "#{path.basename}: duplicate permalink `#{permalink}` also used by #{projects[permalink][:file]}")
    next
  end

  projects[permalink] = {
    file: path.basename.to_s,
    front_matter: front_matter
  }
end

raw_cards = load_yaml_file(PROJECT_CARDS_PATH, PROJECT_CARDS_PATH.basename.to_s)
unless raw_cards.is_a?(Array)
  raise "#{PROJECT_CARDS_PATH.basename}: expected a top-level array of card entries"
end

cards = {}

raw_cards.each_with_index do |card, index|
  label = "#{PROJECT_CARDS_PATH.basename} entry #{index + 1}"

  unless card.is_a?(Hash)
    add_error(errors, "#{label}: expected a mapping")
    next
  end

  url = card["url"]

  if blank?(url)
    add_error(errors, "#{label}: missing `url`")
    next
  end

  if cards.key?(url)
    add_error(errors, "#{label}: duplicate card url `#{url}`")
    next
  end

  project = projects[url]

  unless project
    add_error(errors, "#{label}: url `#{url}` does not match any project permalink")
    next
  end

  project_front_matter = project[:front_matter]
  cards[url] = card

  filter_categories = normalized_array(card["filter_categories"])
  filter_categories = normalized_array(project_front_matter["filter_categories"]) if filter_categories.empty?

  if filter_categories.empty?
    add_error(errors, "#{project[:file]}: missing filter categories in both project card data and project front matter")
  end

  unknown_filters = filter_categories - ALLOWED_FILTERS
  unless unknown_filters.empty?
    add_error(errors, "#{project[:file]}: unknown filter categories #{unknown_filters.map { |item| "`#{item}`" }.join(', ')}")
  end

  if filter_categories.length != filter_categories.uniq.length
    add_error(errors, "#{project[:file]}: duplicate filter categories detected")
  end

  title = card["card_title"] || project_front_matter["card_title"] || project_front_matter["title"]
  add_error(errors, "#{project[:file]}: missing card title fallback chain") if blank?(title)

  description = card["card_description"] ||
                project_front_matter["card_description"] ||
                project_front_matter["share-description"] ||
                project_front_matter["description"]
  add_error(errors, "#{project[:file]}: missing card description fallback chain") if blank?(description)

  category_label = card["category_label"] || project_front_matter["category_label"]
  add_error(errors, "#{project[:file]}: missing `category_label` in project_cards.yml") if blank?(category_label)

  impacts = card.key?("impacts") ? card["impacts"] : project_front_matter["impacts"]
  unless blank?(impacts)
    unless impacts.is_a?(Array)
      add_error(errors, "#{project[:file]}: impacts must be an array")
    end

    Array(impacts).each_with_index do |impact, impact_index|
      unless impact.is_a?(Hash)
        add_error(errors, "#{project[:file]}: impact #{impact_index + 1} must be a mapping")
        next
      end

      add_error(errors, "#{project[:file]}: impact #{impact_index + 1} missing `value`") if blank?(impact["value"])
      add_error(errors, "#{project[:file]}: impact #{impact_index + 1} missing `label`") if blank?(impact["label"])
    end
  end

  tech_tags = card.key?("tech_tags") ? card["tech_tags"] : project_front_matter["tech_tags"]
  unless blank?(tech_tags)
    unless tech_tags.is_a?(Array)
      add_error(errors, "#{project[:file]}: tech_tags must be an array")
    end

    Array(tech_tags).each_with_index do |tag, tag_index|
      unless tag.is_a?(Hash)
        add_error(errors, "#{project[:file]}: tech tag #{tag_index + 1} must be a mapping")
        next
      end

      add_error(errors, "#{project[:file]}: tech tag #{tag_index + 1} missing `label`") if blank?(tag["label"])
      add_error(errors, "#{project[:file]}: tech tag #{tag_index + 1} missing `style`") if blank?(tag["style"])
    end
  end

  media = card["media"]

  if present?(media)
    unless media.is_a?(Hash)
      add_error(errors, "#{project[:file]}: card media override must be a mapping")
      next
    end

    media_type = media["type"]
    media_src = media["src"]
    media_href = media["href"]

    unless %w[image video].include?(media_type)
      add_error(errors, "#{project[:file]}: media.type must be `image` or `video`")
    end

    add_error(errors, "#{project[:file]}: media override missing `src`") if blank?(media_src)
    add_error(errors, "#{project[:file]}: media override missing `href`") if blank?(media_href)
  else
    project_has_media = present?(project_front_matter["video_url"]) || present?(project_front_matter["thumbnail-img"])
    add_error(errors, "#{project[:file]}: no media override and no project-level `video_url` or `thumbnail-img` fallback") unless project_has_media
  end
end

missing_cards = projects.keys - cards.keys
missing_cards.each do |url|
  add_error(errors, "#{projects[url][:file]}: missing matching entry in #{PROJECT_CARDS_PATH.basename}")
end

if errors.any?
  warn "Project card validation failed:"
  errors.each { |message| warn "- #{message}" }
  exit 1
end

puts "Validated #{projects.length} projects and #{cards.length} project card entries."
